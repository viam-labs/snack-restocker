package snackrestocker

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
)

func phases(report map[string]interface{}) []string {
	var out []string
	for _, e := range report["events"].([]interface{}) {
		out = append(out, e.(map[string]interface{})["phase"].(string))
	}
	return out
}

func TestPickStatusBeforeAnyPick(t *testing.T) {
	rig := newPickRig(t, twoBags())
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"get_pick_status": true})
	if resp["phase"] != "none" || resp["running"] != false {
		t.Fatalf("status: %v", resp)
	}
}

func TestPickStatusIsLiveDuringThePick(t *testing.T) {
	rig := newPickRig(t, twoBags())
	var seen []map[string]interface{}
	// The fake planner runs mid-pick, so it can read the status as a poller would.
	rig.refuse = func(p r3.Vector, a graspAngle) bool {
		seen = append(seen, rig.s.pickStatusReport(false))
		return p.Z == 80 // bag 1's descend is refused at both rotations
	}
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" || resp["bag"] != 2 {
		t.Fatalf("pick: %v", resp)
	}

	first := seen[0]
	if first["phase"] != "hovering" || first["running"] != true || first["plausible"] != 2 {
		t.Fatalf("first move should be a live hover over bag 1: %v", first)
	}
	if target := first["target"].(map[string]interface{}); target["bag"] != 1 || target["confidence"] != 0.9 {
		t.Fatalf("target before moving: %v", target)
	}
	// By bag 2's hover, bag 1's two refusals are already visible.
	var bag2 map[string]interface{}
	for _, st := range seen {
		if target, _ := st["target"].(map[string]interface{}); target != nil && target["bag"] == 2 {
			bag2 = st
			break
		}
	}
	if bag2 == nil || bag2["attempts"] != 2 || bag2["last_refusal"].(map[string]interface{})["stage"] != "descend" {
		t.Fatalf("refusals should show before the pick ends: %v", bag2)
	}

	final, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"get_pick_status": true})
	if final["phase"] != "complete" || final["running"] != false || final["command"] != "pick_snack" {
		t.Fatalf("final status: %v", final)
	}
	want := []string{
		"moving_to_view", "segmenting", "targets_found",
		"hovering", "descending", "descend_refused", "hovering", "descending", "descend_refused",
		"hovering", "descending", "gripping", "lifting", "complete",
	}
	got := phases(final)
	if len(got) != len(want) {
		t.Fatalf("events: %v", got)
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("events: %v", got)
		}
	}
}

func TestPickStatusWhenNothingIsFound(t *testing.T) {
	rig := newPickRig(t, map[string]interface{}{"objects": []interface{}{}})
	rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	st := rig.s.pickStatusReport(true)
	if st["phase"] != "failed" || st["running"] != false || st["message"] != "no snack bags found" {
		t.Fatalf("status: %v", st)
	}
	if got := phases(st); got[len(got)-2] != "no_target" {
		t.Fatalf("events: %v", got)
	}
}

func TestPickStatusNewRunClearsTheOldOne(t *testing.T) {
	rig := newPickRig(t, twoBags())
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return p.Z < 100 }
	rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	rig.refuse = nil
	rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "dry_run": true})
	st := rig.s.pickStatusReport(true)
	if st["run"] != 2 || st["phase"] != "planned" || st["attempts"] != 0 || st["last_refusal"] != nil {
		t.Fatalf("second run should not carry the first run's failures: %v", st)
	}
}

func TestPickAndPlaceStatusTracksRounds(t *testing.T) {
	rig := withPlacePoses(newPickRig(t, twoBags()))
	rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_and_place": true})
	st := rig.s.pickStatusReport(true)
	if st["command"] != "pick_and_place" || st["phase"] != "complete" || st["rounds_completed"] != 2 || st["round"] != 2 {
		t.Fatalf("status: %v", st)
	}
	got := phases(st)
	count := map[string]int{}
	for _, p := range got {
		count[p]++
	}
	if count["placing"] != 4 || count["dropping"] != 2 || count["round_complete"] != 2 || count["lifting"] != 2 {
		t.Fatalf("events: %v", got)
	}
}

func TestPickStatusSensorReadings(t *testing.T) {
	rig := newPickRig(t, twoBags())
	rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "dry_run": true})
	sensor := &pickStatusSensor{restocker: rig.s}
	readings, err := sensor.Readings(context.Background(), nil)
	if err != nil {
		t.Fatal(err)
	}
	if readings["phase"] != "planned" || readings["event_count"] == 0 {
		t.Fatalf("readings: %v", readings)
	}
	if _, ok := readings["events"]; ok {
		t.Fatalf("readings should leave out the event history: %v", readings)
	}
}

func TestPickStatusConfigNeedsRestocker(t *testing.T) {
	if _, _, err := (&PickStatusConfig{}).Validate("path"); err == nil {
		t.Fatal("missing restocker should be an error")
	}
	deps, _, err := (&PickStatusConfig{Restocker: "code-1"}).Validate("path")
	if err != nil || len(deps) != 1 || deps[0] != "code-1" {
		t.Fatalf("deps=%v err=%v", deps, err)
	}
}
