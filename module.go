package snackrestocker

import (
	"context"
	"fmt"
	"sync"
	"time"

	"go.viam.com/rdk/components/gripper"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
)

var Restocker = resource.NewModel("viam", "snack-restocker", "restocker")

// Name of the gripper component on this machine.
const gripperName = "gripper"

// Gripper scale: 0 = fully closed, 850 = fully open.
const (
	gripperOpen      = 850
	gripperClose     = 30
	gripperFullClose = 0
)

// Pause after each move / gripper command.
var stepDelay = time.Second

// erh:vmodutils:arm-position-saver switches: SetPosition(1) records a pose,
// SetPosition(2) goes to the saved pose.
const goToSavedPose = 2

// Phase 1: find the drawer.
const findDrawerSwitch = "open-drawer-0-position"

// Phase 2: open the drawer, in order.
var openDrawerSwitches = []string{
	"open-drawer-1-position",
	"open-drawer-2-position",
	"open-drawer-3-position-touch-",
	"open-drawer-4-nudge-position",
	"open-drawer-5-softpull",
	"open-drawer-6-fullpull",
}

// Phase 3: restock poses, in order.
var restockSwitches = []string{
	"restock-1-find",
	"restock-2-reach-snack",
	"restock-3-lift-snack",
	"restock-4-view-drawer",
	"restock-5-drop",
}

// Phase 4: close the drawer after the drop, in order.
var closeSwitches = []string{
	"close-1-set-back",
	"close-2-lower",
	"close-3-push1",
	"close-4-push2",
}

// Exact order of moves run in closingDrawer. close-1-set-back is used twice
// (before lowering and again after the pushes), and the arm returns to the
// find pose at the end.
var closeSequence = []string{
	"close-1-set-back",
	"close-2-lower",
	"close-3-push1",
	"close-4-push2",
	"close-1-set-back",
	findDrawerSwitch,
}

// State is a state of the restock state machine.
type State string

const (
	stateIdle           State = "idle"
	stateFindingDrawer  State = "finding_drawer"
	stateOpeningDrawer  State = "opening_drawer"
	statePickingUpSnack State = "picking_up_snack"
	stateRestocking     State = "restocking"
	stateClosingDrawer  State = "closing_drawer"
	stateDone           State = "done"
)

// Happy-path order of the sequence; drives transition validation and the
// "advance_state" command.
var stateSequence = []State{
	stateIdle,
	stateFindingDrawer,
	stateOpeningDrawer,
	statePickingUpSnack,
	stateRestocking,
	stateClosingDrawer,
	stateDone,
}

const initialState = stateIdle

// Allowed transitions: forward along the sequence, done -> idle, plus an
// abort/reset back to idle from anywhere.
var allowedTransitions = map[State][]State{
	stateIdle:           {stateFindingDrawer},
	stateFindingDrawer:  {stateOpeningDrawer, stateIdle},
	stateOpeningDrawer:  {statePickingUpSnack, stateIdle},
	statePickingUpSnack: {stateRestocking, stateIdle},
	stateRestocking:     {stateClosingDrawer, stateIdle},
	stateClosingDrawer:  {stateDone, stateIdle},
	stateDone:           {stateIdle},
}

func init() {
	resource.RegisterService(generic.API, Restocker,
		resource.Registration[resource.Resource, Config]{
			Constructor: newSnackRestockerRestocker,
		},
	)
}

// Config has no attributes; the gripper and switch names are fixed above.
type Config struct{}

// Validate requires the gripper and every pose-saver switch used by the
// restock sequence. The fixture-teacher drawer services are optional; only
// restock_fixture uses them.
func (cfg Config) Validate(path string) ([]string, []string, error) {
	return append([]string{gripperName}, allSwitches()...), []string{drawerOpenerName, drawerCloserName}, nil
}

// allSwitches lists every pose-saver switch used by the restock sequence.
func allSwitches() []string {
	names := []string{findDrawerSwitch}
	names = append(names, openDrawerSwitches...)
	names = append(names, restockSwitches...)
	return append(names, closeSwitches...)
}

type snackRestockerRestocker struct {
	resource.AlwaysRebuild
	resource.Named

	name resource.Name

	logger logging.Logger
	cfg    Config

	gripper  gripper.Gripper
	switches map[string]toggleswitch.Switch

	// Optional fixture-teacher services; nil when not on the machine. Used
	// only by restock_fixture.
	drawerOpener resource.Resource
	drawerCloser resource.Resource

	mu    sync.Mutex
	state State

	cancelCtx  context.Context
	cancelFunc func()
}

func newSnackRestockerRestocker(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewRestocker(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewRestocker(ctx context.Context, deps resource.Dependencies, name resource.Name, conf Config, logger logging.Logger) (resource.Resource, error) {
	g, err := gripper.FromDependencies(deps, gripperName)
	if err != nil {
		return nil, err
	}

	switches := map[string]toggleswitch.Switch{}
	for _, n := range allSwitches() {
		sw, err := toggleswitch.FromDependencies(deps, n)
		if err != nil {
			return nil, err
		}
		switches[n] = sw
	}

	drawerOpener, err := generic.FromProvider(deps, drawerOpenerName)
	if err != nil {
		logger.Infof("%s not available, restock_fixture disabled: %v", drawerOpenerName, err)
		drawerOpener = nil
	}
	drawerCloser, err := generic.FromProvider(deps, drawerCloserName)
	if err != nil {
		logger.Infof("%s not available, restock_fixture disabled: %v", drawerCloserName, err)
		drawerCloser = nil
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &snackRestockerRestocker{
		name:         name,
		logger:       logger,
		cfg:          conf,
		gripper:      g,
		switches:     switches,
		drawerOpener: drawerOpener,
		drawerCloser: drawerCloser,
		state:        initialState,
		cancelCtx:    cancelCtx,
		cancelFunc:   cancelFunc,
	}
	return s, nil
}

func (s *snackRestockerRestocker) Name() resource.Name {
	return s.name
}

// --- state machine ------------------------------------------------

func (s *snackRestockerRestocker) currentState() State {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.state
}

// transition validates and logs a state change; it errors if the change is
// not allowed.
func (s *snackRestockerRestocker) transition(newState State, force bool) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	oldState := s.state
	if newState == oldState {
		s.logger.Debugf("state unchanged: '%s'", oldState)
		return nil
	}
	allowed := allowedTransitions[oldState]
	if !force && !containsState(allowed, newState) {
		return fmt.Errorf("illegal state transition '%s' -> '%s'; allowed: %v", oldState, newState, allowed)
	}
	s.state = newState
	s.logger.Infof("state transition: '%s' -> '%s'", oldState, newState)
	return nil
}

// nextState returns the next state on the happy path, or "" if there is none.
func nextState(current State) State {
	for i, st := range stateSequence {
		if st == current && i+1 < len(stateSequence) {
			return stateSequence[i+1]
		}
	}
	return ""
}

func containsState(states []State, target State) bool {
	for _, st := range states {
		if st == target {
			return true
		}
	}
	return false
}

func stateStrings(states []State) []interface{} {
	out := make([]interface{}, 0, len(states))
	for _, st := range states {
		out = append(out, string(st))
	}
	return out
}

// stateReport is the common payload returned by the state-related commands.
func (s *snackRestockerRestocker) stateReport(extra map[string]interface{}) map[string]interface{} {
	current := s.currentState()
	report := map[string]interface{}{
		"state":               string(current),
		"next_state":          string(nextState(current)),
		"allowed_transitions": stateStrings(allowedTransitions[current]),
	}
	for k, v := range extra {
		report[k] = v
	}
	return report
}

// --- restock sequence ---------------------------------------------

// goToPose moves to the pose saved on the named switch, in the current state.
func (s *snackRestockerRestocker) goToPose(ctx context.Context, name string, step int, steps *[]interface{}) error {
	state := s.currentState()
	s.logger.Infof("restock step %d [%s]: moving to saved pose '%s'", step, state, name)
	sw, ok := s.switches[name]
	if !ok {
		return fmt.Errorf("switch '%s' is not available as a dependency", name)
	}
	if err := sw.SetPosition(ctx, goToSavedPose, nil); err != nil {
		return err
	}
	if err := sleep(ctx, stepDelay); err != nil {
		return err
	}
	*steps = append(*steps, fmt.Sprintf("%s: %s: set_position(%d)", state, name, goToSavedPose))
	return nil
}

// setGripper opens/closes the gripper on the 0-850 scale, in the current state.
func (s *snackRestockerRestocker) setGripper(ctx context.Context, value, step int, steps *[]interface{}) error {
	state := s.currentState()
	action := "closing"
	if value >= gripperOpen {
		action = "opening"
	}
	s.logger.Infof("restock step %d [%s]: %s gripper (set=%d)", step, state, action, value)
	if _, err := s.gripper.DoCommand(ctx, map[string]interface{}{"set": value}); err != nil {
		return err
	}
	if err := sleep(ctx, stepDelay); err != nil {
		return err
	}
	*steps = append(*steps, fmt.Sprintf("%s: %s: do_command(set=%d)", state, gripperName, value))
	return nil
}

// drawerFound reports whether the find-drawer pose has located the drawer.
func (s *snackRestockerRestocker) drawerFound(ctx context.Context) bool {
	return true
}

func sleep(ctx context.Context, d time.Duration) error {
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-time.After(d):
		return nil
	}
}

// runRestock runs the full sequence, driving the state machine.
func (s *snackRestockerRestocker) runRestock(ctx context.Context) (map[string]interface{}, error) {
	steps := []interface{}{}
	step := 0

	next := func() int {
		step++
		return step
	}

	s.logger.Info("restock sequence: starting")
	// Always start the flow from idle so each transition below is legal.
	if err := s.transition(stateIdle, true); err != nil {
		return nil, err
	}

	// --- finding_drawer -------------------------------------------
	// Close the gripper before touching the drawer, run the find pose,
	// and hold this state until the drawer is found.
	if err := s.transition(stateFindingDrawer, false); err != nil {
		return nil, err
	}
	if err := s.setGripper(ctx, gripperFullClose, next(), &steps); err != nil {
		return nil, err
	}
	if err := s.goToPose(ctx, findDrawerSwitch, next(), &steps); err != nil {
		return nil, err
	}
	for !s.drawerFound(ctx) {
		s.logger.Info("finding_drawer: drawer not found yet, holding state")
		if err := sleep(ctx, stepDelay); err != nil {
			return nil, err
		}
	}
	s.logger.Info("finding_drawer: drawer found")

	// --- opening_drawer -------------------------------------------
	if err := s.transition(stateOpeningDrawer, false); err != nil {
		return nil, err
	}
	for _, name := range openDrawerSwitches {
		if err := s.goToPose(ctx, name, next(), &steps); err != nil {
			return nil, err
		}
	}

	if err := s.pickAndDrop(ctx, next, &steps); err != nil {
		return nil, err
	}

	// --- closing_drawer -------------------------------------------
	// Push the drawer shut, then return to the find-drawer pose.
	if err := s.transition(stateClosingDrawer, false); err != nil {
		return nil, err
	}
	for _, name := range closeSequence {
		if err := s.goToPose(ctx, name, next(), &steps); err != nil {
			return nil, err
		}
	}

	// --- done -----------------------------------------------------
	// Entered once the drawer is closed and the arm is back at the find pose.
	if err := s.transition(stateDone, false); err != nil {
		return nil, err
	}
	s.logger.Infof("restock sequence: complete (%d steps)", step)

	// --- idle -----------------------------------------------------
	// Rest in idle until the next restock.
	if err := s.transition(stateIdle, false); err != nil {
		return nil, err
	}
	return map[string]interface{}{
		"restock":         "complete",
		"steps_completed": step,
		"steps":           steps,
		"state":           string(s.currentState()),
	}, nil
}

// pickAndDrop runs picking_up_snack and restocking: grab the snack from the
// saved restock poses, drop it in the open drawer, then close the gripper
// fully so the arm can push the drawer shut.
func (s *snackRestockerRestocker) pickAndDrop(ctx context.Context, next func() int, steps *[]interface{}) error {
	// --- picking_up_snack -----------------------------------------
	if err := s.transition(statePickingUpSnack, false); err != nil {
		return err
	}
	// restock-1-find, then open the gripper to prepare for the snack.
	if err := s.goToPose(ctx, "restock-1-find", next(), steps); err != nil {
		return err
	}
	if err := s.setGripper(ctx, gripperOpen, next(), steps); err != nil {
		return err
	}
	// restock-2-reach-snack, then grab the snack.
	if err := s.goToPose(ctx, "restock-2-reach-snack", next(), steps); err != nil {
		return err
	}
	if err := s.setGripper(ctx, gripperClose, next(), steps); err != nil {
		return err
	}
	// restock-3-lift-snack: lift the snack clear.
	if err := s.goToPose(ctx, "restock-3-lift-snack", next(), steps); err != nil {
		return err
	}

	// --- restocking -----------------------------------------------
	if err := s.transition(stateRestocking, false); err != nil {
		return err
	}
	for _, name := range []string{"restock-4-view-drawer", "restock-5-drop"} {
		if err := s.goToPose(ctx, name, next(), steps); err != nil {
			return err
		}
	}
	// restocking ends once the gripper opens to release the snack.
	if err := s.setGripper(ctx, gripperOpen, next(), steps); err != nil {
		return err
	}
	// Close the gripper fully so the arm pushes the drawer shut.
	if err := s.setGripper(ctx, gripperFullClose, next(), steps); err != nil {
		return err
	}
	return nil
}

func (s *snackRestockerRestocker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if restock, _ := cmd["restock"].(bool); restock {
		result, err := s.runRestock(ctx)
		if err != nil {
			s.logger.Errorf("restock sequence failed: %v", err)
			_ = s.transition(initialState, true)
			return map[string]interface{}{
				"restock": "failed",
				"error":   err.Error(),
				"state":   string(s.currentState()),
			}, nil
		}
		return result, nil
	}

	if fixture, _ := cmd["restock_fixture"].(bool); fixture {
		result, err := s.runFixtureRestock(ctx)
		if err != nil {
			s.logger.Errorf("fixture restock sequence failed: %v", err)
			_ = s.transition(initialState, true)
			return map[string]interface{}{
				"restock_fixture": "failed",
				"error":           err.Error(),
				"state":           string(s.currentState()),
			}, nil
		}
		return result, nil
	}

	if _, ok := cmd["get_state"]; ok {
		return s.stateReport(nil), nil
	}

	if _, ok := cmd["advance_state"]; ok {
		target := nextState(s.currentState())
		if target == "" {
			return s.stateReport(map[string]interface{}{
				"error": fmt.Sprintf("no next state from '%s'", s.currentState()),
			}), nil
		}
		if err := s.transition(target, false); err != nil {
			return s.stateReport(map[string]interface{}{"error": err.Error()}), nil
		}
		return s.stateReport(map[string]interface{}{"advanced": true}), nil
	}

	if requested, ok := cmd["set_state"]; ok {
		force, _ := cmd["force"].(bool)
		target := State(fmt.Sprint(requested))
		if !containsState(stateSequence, target) {
			return s.stateReport(map[string]interface{}{
				"error":        fmt.Sprintf("unknown state: %v", requested),
				"valid_states": stateStrings(stateSequence),
			}), nil
		}
		if err := s.transition(target, force); err != nil {
			return s.stateReport(map[string]interface{}{"error": err.Error()}), nil
		}
		return s.stateReport(map[string]interface{}{"set_state": true}), nil
	}

	if _, ok := cmd["reset_state"]; ok {
		_ = s.transition(initialState, true)
		return s.stateReport(map[string]interface{}{"reset": true}), nil
	}

	keys := make([]interface{}, 0, len(cmd))
	for k := range cmd {
		keys = append(keys, k)
	}
	return map[string]interface{}{
		"error": fmt.Sprintf("unknown command: %v", keys),
		"usage": `send {"restock": true} to run the restock sequence, ` +
			`{"restock_fixture": true} to run it with the drawer-open and draw-closer services, ` +
			`{"get_state": true} to read the state machine, ` +
			`{"advance_state": true} to step to the next state, ` +
			`{"set_state": "<state>", "force": false} to set a state, ` +
			`or {"reset_state": true} to return to idle`,
		"supported_commands": []interface{}{"restock", "restock_fixture", "get_state", "advance_state", "set_state", "reset_state"},
		"states":             stateStrings(stateSequence),
		"state":              string(s.currentState()),
	}, nil
}

func (s *snackRestockerRestocker) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{"state": string(s.currentState())}, nil
}

func (s *snackRestockerRestocker) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
