package snackrestocker

import (
	"context"
	"fmt"

	"go.viam.com/rdk/resource"
)

// Fixture restock: the same restock sequence, but the drawer is opened and
// closed by viam-labs:restock-bot:fixture-teacher services, which replay
// motions taught relative to the drawer's AprilTag, instead of the saved
// open-drawer-* and close-* poses. Runs from the restock_fixture DoCommand;
// the restock command is unchanged.

// Optional dependencies used only by restock_fixture.
const (
	drawerOpenerName = "drawer-open"
	drawerCloserName = "draw-closer"
)

// Taught motion names on the fixture-teacher services.
const (
	openMotion  = "open"
	closeMotion = "close"
)

// runFixtureMotion replays a taught motion on a fixture-teacher service. The
// service reports failures in an "error" field rather than an RPC error.
func (s *snackRestockerRestocker) runFixtureMotion(
	ctx context.Context, svc resource.Resource, svcName, motionName string, step int, steps *[]interface{},
) error {
	state := s.currentState()
	if svc == nil {
		return fmt.Errorf("'%s' fixture service is not available on the machine", svcName)
	}
	s.logger.Infof("fixture restock step %d [%s]: %s run motion '%s'", step, state, svcName, motionName)
	resp, err := svc.DoCommand(ctx, map[string]interface{}{"cmd": "run", "motion": motionName})
	if err != nil {
		return fmt.Errorf("%s run %s: %w", svcName, motionName, err)
	}
	if msg, ok := resp["error"]; ok && msg != nil && msg != "" {
		if failed, ok := resp["failed_step"]; ok {
			return fmt.Errorf("%s run %s failed at step %v: %v", svcName, motionName, failed, msg)
		}
		return fmt.Errorf("%s run %s: %v", svcName, motionName, msg)
	}
	if ok, isBool := resp["ok"].(bool); isBool && !ok {
		return fmt.Errorf("%s run %s did not succeed: %v", svcName, motionName, resp)
	}
	if err := sleep(ctx, stepDelay); err != nil {
		return err
	}
	*steps = append(*steps, fmt.Sprintf("%s: %s: run(%s)", state, svcName, motionName))
	return nil
}

// runFixtureRestock runs the restock sequence with fixture-teacher drawer
// motions, driving the same state machine as runRestock.
func (s *snackRestockerRestocker) runFixtureRestock(ctx context.Context) (map[string]interface{}, error) {
	steps := []interface{}{}
	step := 0

	next := func() int {
		step++
		return step
	}

	s.logger.Info("fixture restock sequence: starting")
	if err := s.transition(stateIdle, true); err != nil {
		return nil, err
	}

	// --- finding_drawer -------------------------------------------
	// Close the gripper before touching the drawer. drawer-open moves to its
	// own view pose and locates the drawer marker itself.
	if err := s.transition(stateFindingDrawer, false); err != nil {
		return nil, err
	}
	if err := s.setGripper(ctx, gripperFullClose, next(), &steps); err != nil {
		return nil, err
	}

	// --- opening_drawer -------------------------------------------
	if err := s.transition(stateOpeningDrawer, false); err != nil {
		return nil, err
	}
	if err := s.runFixtureMotion(ctx, s.drawerOpener, drawerOpenerName, openMotion, next(), &steps); err != nil {
		return nil, err
	}

	// --- picking_up_snack, restocking -----------------------------
	if err := s.pickAndDrop(ctx, next, &steps); err != nil {
		return nil, err
	}

	// --- closing_drawer -------------------------------------------
	// Push the drawer shut, then return to the find-drawer pose.
	if err := s.transition(stateClosingDrawer, false); err != nil {
		return nil, err
	}
	if err := s.runFixtureMotion(ctx, s.drawerCloser, drawerCloserName, closeMotion, next(), &steps); err != nil {
		return nil, err
	}
	if err := s.goToPose(ctx, findDrawerSwitch, next(), &steps); err != nil {
		return nil, err
	}

	// --- done, idle -----------------------------------------------
	if err := s.transition(stateDone, false); err != nil {
		return nil, err
	}
	s.logger.Infof("fixture restock sequence: complete (%d steps)", step)
	if err := s.transition(stateIdle, false); err != nil {
		return nil, err
	}
	return map[string]interface{}{
		"restock_fixture": "complete",
		"steps_completed": step,
		"steps":           steps,
		"state":           string(s.currentState()),
	}, nil
}
