package snackrestocker

import (
	"context"
	"testing"

	"go.viam.com/rdk/logging"
)

func TestStateCommands(t *testing.T) {
	ctx := context.Background()
	s := &snackRestockerRestocker{logger: logging.NewTestLogger(t), state: initialState, cancelFunc: func() {}}

	resp, _ := s.DoCommand(ctx, map[string]interface{}{"advance_state": true})
	if resp["state"] != "finding_drawer" || resp["advanced"] != true {
		t.Fatalf("advance_state: %v", resp)
	}

	resp, _ = s.DoCommand(ctx, map[string]interface{}{"set_state": "done"})
	if resp["error"] == nil || resp["state"] != "finding_drawer" {
		t.Fatalf("illegal set_state should fail: %v", resp)
	}

	resp, _ = s.DoCommand(ctx, map[string]interface{}{"set_state": "done", "force": true})
	if resp["state"] != "done" || resp["next_state"] != "" {
		t.Fatalf("forced set_state: %v", resp)
	}

	resp, _ = s.DoCommand(ctx, map[string]interface{}{"set_state": "bogus"})
	if resp["valid_states"] == nil {
		t.Fatalf("unknown state: %v", resp)
	}

	resp, _ = s.DoCommand(ctx, map[string]interface{}{"reset_state": true})
	if resp["state"] != "idle" {
		t.Fatalf("reset_state: %v", resp)
	}

	resp, _ = s.DoCommand(ctx, map[string]interface{}{"nope": true})
	if resp["supported_commands"] == nil {
		t.Fatalf("usage: %v", resp)
	}
}
