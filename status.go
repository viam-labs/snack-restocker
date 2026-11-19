package snackrestocker

import (
	"sync"
	"time"
)

// Live status of the current or last pick_snack / pick_and_place run, readable
// with the get_pick_status DoCommand (and the pick-status sensor) while the
// run is still moving the arm.

const maxStatusEvents = 50

type pickStatus struct {
	mu     sync.Mutex
	run    int
	fields map[string]interface{}
	events []interface{}
	since  time.Time
}

func statusTime(t time.Time) string {
	return t.UTC().Format(time.RFC3339Nano)
}

// statusStart begins a new run, clearing the previous run's status.
func (s *snackRestockerRestocker) statusStart(command string) {
	p := &s.pickStatus
	p.mu.Lock()
	defer p.mu.Unlock()
	now := time.Now()
	p.run++
	p.since = now
	p.events = nil
	p.fields = map[string]interface{}{
		"run":        p.run,
		"command":    command,
		"running":    true,
		"phase":      "starting",
		"message":    "",
		"started_at": statusTime(now),
		"updated_at": statusTime(now),
	}
}

// statusEvent moves the run to phase, records it as an event, and merges set
// into the status. It is a no-op outside a run.
func (s *snackRestockerRestocker) statusEvent(phase, message string, set map[string]interface{}) {
	p := &s.pickStatus
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.fields == nil {
		return
	}
	now := time.Now()
	if phase != "" && phase != p.fields["phase"] {
		p.since = now
	}
	if phase != "" {
		p.fields["phase"] = phase
		p.fields["message"] = message
		p.events = append(p.events, map[string]interface{}{
			"at": statusTime(now), "phase": phase, "message": message,
		})
		if len(p.events) > maxStatusEvents {
			p.events = p.events[len(p.events)-maxStatusEvents:]
		}
	}
	for k, v := range set {
		p.fields[k] = v
	}
	p.fields["updated_at"] = statusTime(now)
}

// statusFinish ends the run with a final phase such as complete or failed.
func (s *snackRestockerRestocker) statusFinish(phase, message string) {
	s.statusEvent(phase, message, map[string]interface{}{"running": false})
}

// finishFromResult ends the run using the status a pick command returned,
// e.g. result["pick_snack"] == "complete".
func (s *snackRestockerRestocker) finishFromResult(result map[string]interface{}, key string) {
	phase, _ := result[key].(string)
	if phase == "" {
		phase = "done"
	}
	message, _ := result["error"].(string)
	s.statusFinish(phase, message)
}

// pickStatusReport is a copy of the status plus its event history.
func (s *snackRestockerRestocker) pickStatusReport(withEvents bool) map[string]interface{} {
	p := &s.pickStatus
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.fields == nil {
		return map[string]interface{}{"running": false, "phase": "none", "message": "no pick has run yet"}
	}
	report := make(map[string]interface{}, len(p.fields)+3)
	for k, v := range p.fields {
		report[k] = v
	}
	report["seconds_in_phase"] = time.Since(p.since).Seconds()
	report["event_count"] = len(p.events)
	if withEvents {
		report["events"] = append([]interface{}{}, p.events...)
	}
	return report
}
