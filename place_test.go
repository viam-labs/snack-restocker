package snackrestocker

import (
	"context"
	"reflect"
	"testing"

	"github.com/golang/geo/r3"

	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/testutils/inject"
)

// withPlacePoses adds the drop pose switches to a pick rig, recording visits.
func withPlacePoses(rig *pickRig) *pickRig {
	for _, name := range placePoses {
		name := name
		sw := inject.NewSwitch(name)
		sw.SetPositionFunc = func(ctx context.Context, position uint32, extra map[string]interface{}) error {
			rig.visited = append(rig.visited, name)
			return nil
		}
		rig.s.switches[name] = toggleswitch.Switch(sw)
	}
	return rig
}

func TestPickAndPlaceRunsTwoRounds(t *testing.T) {
	rig := withPlacePoses(newPickRig(t, twoBags(t)))
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_and_place": true})
	if resp["pick_and_place"] != "complete" || resp["rounds_completed"] != 2 {
		t.Fatalf("pick_and_place: %v", resp)
	}
	round := []string{pickViewSwitch, "restock-4-view-drawer", "restock-5-drop"}
	if want := append(append([]string{}, round...), round...); !reflect.DeepEqual(rig.visited, want) {
		t.Fatalf("visited %v, want %v", rig.visited, want)
	}
	// Each round: open, close on the bag (pick_snack), then open to drop.
	wantGrips := []interface{}{gripperOpen, gripperClose, gripperOpen, gripperOpen, gripperClose, gripperOpen}
	if !reflect.DeepEqual(rig.grips, wantGrips) {
		t.Fatalf("gripper %v, want %v", rig.grips, wantGrips)
	}
	// Hover, descend, lift per round.
	if len(rig.moves) != 6 {
		t.Fatalf("moves: %v", rig.moves)
	}
	if rig.s.currentState() != initialState {
		t.Fatalf("pick_and_place must not touch the restock state machine: %s", rig.s.currentState())
	}
}

func TestPickAndPlaceRounds(t *testing.T) {
	rig := withPlacePoses(newPickRig(t, twoBags(t)))
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_and_place": true, "rounds": 3.0})
	if resp["pick_and_place"] != "complete" || resp["rounds_completed"] != 3 || len(rig.visited) != 9 {
		t.Fatalf("resp=%v visited=%v", resp, rig.visited)
	}
}

func TestPickAndPlaceStopsWhenPickFails(t *testing.T) {
	rig := withPlacePoses(newPickRig(t, twoBags(t)))
	rig.refuse = func(_ r3.Vector, _ graspAngle) bool { return true }
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_and_place": true})
	if resp["pick_and_place"] != "failed" || resp["rounds_completed"] != 0 || resp["error"] == nil {
		t.Fatalf("pick_and_place: %v", resp)
	}
	if !reflect.DeepEqual(rig.visited, []string{pickViewSwitch}) {
		t.Fatalf("should not go to the drawer after a failed pick: %v", rig.visited)
	}
}

func TestPickAndPlaceDryRunDoesNotPlace(t *testing.T) {
	rig := withPlacePoses(newPickRig(t, twoBags(t)))
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_and_place": true, "dry_run": true})
	if resp["pick_and_place"] != "planned" || len(rig.moves) != 0 || len(rig.grips) != 0 {
		t.Fatalf("resp=%v moves=%v grips=%v", resp, rig.moves, rig.grips)
	}
}
