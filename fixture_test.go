package snackrestocker

import (
	"context"
	"fmt"
	"reflect"
	"testing"
	"time"

	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/testutils/inject"
)

// newFakeRestocker wires fake switches, gripper and drawer services that
// append every call to calls, in order.
func newFakeRestocker(t *testing.T, openResp, closeResp map[string]interface{}) (*snackRestockerRestocker, *[]string) {
	stepDelay = 0
	t.Cleanup(func() { stepDelay = time.Second })

	calls := []string{}
	switches := map[string]toggleswitch.Switch{}
	for _, name := range allSwitches() {
		sw := inject.NewSwitch(name)
		swName := name
		sw.SetPositionFunc = func(ctx context.Context, position uint32, extra map[string]interface{}) error {
			calls = append(calls, swName)
			return nil
		}
		switches[name] = sw
	}
	g := inject.NewGripper(gripperName)
	g.DoFunc = func(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
		calls = append(calls, fmt.Sprintf("gripper=%v", cmd["set"]))
		return nil, nil
	}
	fixture := func(name string, resp map[string]interface{}) *inject.GenericService {
		svc := inject.NewGenericService(name)
		svc.DoFunc = func(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
			if cmd["cmd"] != "run" {
				t.Fatalf("%s: unexpected command %v", name, cmd)
			}
			calls = append(calls, fmt.Sprintf("%s:%v", name, cmd["motion"]))
			return resp, nil
		}
		return svc
	}
	s := &snackRestockerRestocker{
		logger:       logging.NewTestLogger(t),
		gripper:      g,
		switches:     switches,
		drawerOpener: fixture(drawerOpenerName, openResp),
		drawerCloser: fixture(drawerCloserName, closeResp),
		state:        initialState,
		cancelFunc:   func() {},
	}
	return s, &calls
}

var okRun = map[string]interface{}{"ok": true, "steps": []interface{}{"approach", "contact"}}

// The snack pick and drop, shared by both flows.
var pickAndDropCalls = []string{
	"restock-1-find", "gripper=850", "restock-2-reach-snack", "gripper=30", "restock-3-lift-snack",
	"restock-4-view-drawer", "restock-5-drop", "gripper=850", "gripper=0",
}

func TestRestockFixtureUsesDrawerServices(t *testing.T) {
	s, calls := newFakeRestocker(t, okRun, okRun)
	resp, _ := s.DoCommand(context.Background(), map[string]interface{}{"restock_fixture": true})
	if resp["restock_fixture"] != "complete" || resp["state"] != "idle" {
		t.Fatalf("restock_fixture: %v", resp)
	}
	want := append([]string{"gripper=0", "drawer-open:open"}, pickAndDropCalls...)
	want = append(want, "draw-closer:close", findDrawerSwitch)
	if !reflect.DeepEqual(*calls, want) {
		t.Fatalf("calls:\n got %v\nwant %v", *calls, want)
	}
}

func TestRestockFixtureStopsWhenDrawerFails(t *testing.T) {
	failed := map[string]interface{}{"ok": false, "error": "marker not found", "failed_step": "approach"}
	s, calls := newFakeRestocker(t, failed, okRun)
	resp, _ := s.DoCommand(context.Background(), map[string]interface{}{"restock_fixture": true})
	if resp["restock_fixture"] != "failed" || resp["state"] != "idle" {
		t.Fatalf("should fail and reset: %v", resp)
	}
	if want := []string{"gripper=0", "drawer-open:open"}; !reflect.DeepEqual(*calls, want) {
		t.Fatalf("must stop after the failed open: %v", *calls)
	}
}

func TestRestockFixtureWithoutDrawerServices(t *testing.T) {
	s, calls := newFakeRestocker(t, okRun, okRun)
	s.drawerOpener = nil
	resp, _ := s.DoCommand(context.Background(), map[string]interface{}{"restock_fixture": true})
	if resp["restock_fixture"] != "failed" {
		t.Fatalf("missing drawer-open: %v", resp)
	}
	if len(*calls) != 1 {
		t.Fatalf("only the gripper close should run: %v", *calls)
	}
}

// Guards the original restock flow against changes from sharing pickAndDrop.
func TestRestockSequenceUnchanged(t *testing.T) {
	s, calls := newFakeRestocker(t, okRun, okRun)
	resp, _ := s.DoCommand(context.Background(), map[string]interface{}{"restock": true})
	if resp["restock"] != "complete" || resp["steps_completed"] != 23 {
		t.Fatalf("restock: %v", resp)
	}
	want := append([]string{"gripper=0", findDrawerSwitch}, openDrawerSwitches...)
	want = append(want, pickAndDropCalls...)
	want = append(want, closeSequence...)
	if !reflect.DeepEqual(*calls, want) {
		t.Fatalf("calls:\n got %v\nwant %v", *calls, want)
	}
}
