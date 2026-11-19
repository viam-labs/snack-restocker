package snackrestocker

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

// Vision-guided pick: a standalone DoCommand, separate from the restock
// sequence. It looks at the bin from restock-1-find, asks the SAM3 segmenter
// for the highest-confidence snack bag, and grabs it from above.

// Optional dependencies used only by pick_snack.
const (
	sam3VisionName = "sam3-vision"
	motionName     = "builtin"
)

// Frame moved by the motion planner: the gripper's tool center point.
const tcpFrame = "gripper-tcp"

// Pose the camera looks at the bin from before segmenting.
const pickViewSwitch = "restock-1-find"

// Defaults, overridable per call.
const (
	defaultApproachMm   = 120.0 // hover height above the top of the bag
	defaultGraspDepthMm = 20.0  // how far below the top of the bag to close
)

// Where a real snack bag can be: inside the footprint of the "bin" obstacle
// (centered at (250, -439), 555 x 355 mm) and between the table and the top
// of the bin. Segments outside it come from bad depth readings, such as
// far-away background pixels in a mask.
var (
	binMin = r3.Vector{X: 250 - 555.0/2, Y: -439 - 355.0/2, Z: -50}
	binMax = r3.Vector{X: 250 + 555.0/2, Y: -439 + 355.0/2, Z: 450}
)

// Largest side of a plausible snack bag, in mm. Smaller segments are
// fragments; larger ones include points that are not on the bag.
const (
	minBagSizeMm = 40.0
	maxBagSizeMm = 400.0
)

// Gripper rotations about the vertical to try for each bag, in degrees. 90
// turns the claws parallel to a bin wall the bag is lying against.
var gripperRotations = []float64{0, 90}

// How many plausible bags to try before giving up, by default.
const defaultMaxBags = 3

// Straight-line tolerance for the descend and lift moves.
var straightDown = &motionplan.Constraints{
	LinearConstraint: []motionplan.LinearConstraint{{LineToleranceMm: 10, OrientationToleranceDegs: 5}},
}

// snackTarget is a plausible SAM3 segment, in the world frame.
type snackTarget struct {
	label      string
	confidence float64
	center     r3.Vector
	topZ       float64
}

// topDown is the tool pointing straight down (-Z in world), rotated by
// rotationDeg about the vertical.
func topDown(p r3.Vector, rotationDeg float64) *referenceframe.PoseInFrame {
	return referenceframe.NewPoseInFrame(referenceframe.World,
		spatialmath.NewPose(p, &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: rotationDeg}))
}

func vec(v r3.Vector) map[string]interface{} {
	return map[string]interface{}{"x": v.X, "y": v.Y, "z": v.Z}
}

// findSnacks asks sam3-vision for segments (sorted best-first) and returns
// the ones that could be a real bag in the bin, in the same order, plus the
// total number of segments.
func (s *snackRestockerRestocker) findSnacks(ctx context.Context) ([]*snackTarget, int, error) {
	resp, err := s.vision.DoCommand(ctx, map[string]interface{}{"command": "get_object_point_clouds"})
	if err != nil {
		return nil, 0, fmt.Errorf("%s get_object_point_clouds: %w", sam3VisionName, err)
	}
	if msg, ok := resp["error"]; ok {
		return nil, 0, fmt.Errorf("%s: %v", sam3VisionName, msg)
	}
	objects, _ := resp["objects"].([]interface{})
	if len(objects) == 0 {
		return nil, 0, errors.New("no snack bags found")
	}
	var targets []*snackTarget
	var reasons []string
	for i, raw := range objects {
		target, reason := parseSnack(raw)
		if target == nil {
			s.logger.Debugf("pick: skipping segment %d: %s", i, reason)
			reasons = append(reasons, reason)
			continue
		}
		targets = append(targets, target)
	}
	if len(targets) == 0 {
		return nil, len(objects), fmt.Errorf("none of the %d segments is a plausible bag in the bin; first rejection: %s",
			len(objects), reasons[0])
	}
	return targets, len(objects), nil
}

// parseSnack returns the target for one sam3 segment, or nil and the reason
// it is not a plausible bag in the bin.
func parseSnack(raw interface{}) (*snackTarget, string) {
	obj, ok := raw.(map[string]interface{})
	if !ok {
		return nil, fmt.Sprintf("unexpected object %v", raw)
	}
	if frame := fmt.Sprint(obj["reference_frame"]); frame != referenceframe.World {
		return nil, fmt.Sprintf("frame %q is not world; check the camera's frame config", frame)
	}
	center, err := readVec(obj["center_mm"])
	if err != nil {
		return nil, "center_mm: " + err.Error()
	}
	dims, err := readVec(obj["dims_mm"])
	if err != nil {
		return nil, "dims_mm: " + err.Error()
	}
	maxes, err := readVec(obj["max_mm"])
	if err != nil {
		return nil, "max_mm: " + err.Error()
	}
	if center.X < binMin.X || center.X > binMax.X || center.Y < binMin.Y || center.Y > binMax.Y ||
		center.Z < binMin.Z || center.Z > binMax.Z {
		return nil, fmt.Sprintf("center (%.0f, %.0f, %.0f) is outside the bin", center.X, center.Y, center.Z)
	}
	if size := math.Max(dims.X, math.Max(dims.Y, dims.Z)); size < minBagSizeMm || size > maxBagSizeMm {
		return nil, fmt.Sprintf("largest side %.0f mm is not bag-sized (%.0f-%.0f mm)", size, minBagSizeMm, maxBagSizeMm)
	}
	confidence, _ := obj["confidence"].(float64)
	return &snackTarget{
		label:      fmt.Sprint(obj["label"]),
		confidence: confidence,
		center:     center,
		topZ:       maxes.Z,
	}, ""
}

func readVec(v interface{}) (r3.Vector, error) {
	m, ok := v.(map[string]interface{})
	if !ok {
		return r3.Vector{}, fmt.Errorf("expected {x, y, z}, got %v", v)
	}
	x, okX := m["x"].(float64)
	y, okY := m["y"].(float64)
	z, okZ := m["z"].(float64)
	if !okX || !okY || !okZ {
		return r3.Vector{}, fmt.Errorf("expected numeric x, y, z, got %v", m)
	}
	return r3.Vector{X: x, Y: y, Z: z}, nil
}

// moveTCP plans and runs a move of the gripper TCP to a world pose, pointing
// down and rotated by rotationDeg.
func (s *snackRestockerRestocker) moveTCP(
	ctx context.Context, desc string, p r3.Vector, rotationDeg float64, constraints *motionplan.Constraints,
	step int, steps *[]interface{},
) error {
	s.logger.Infof("pick step %d: %s, moving %s to (%.0f, %.0f, %.0f) mm, rotation %.0f deg",
		step, desc, tcpFrame, p.X, p.Y, p.Z, rotationDeg)
	if _, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: tcpFrame,
		Destination:   topDown(p, rotationDeg),
		Constraints:   constraints,
	}); err != nil {
		return fmt.Errorf("%s: %w", desc, err)
	}
	if err := sleep(ctx, stepDelay); err != nil {
		return err
	}
	*steps = append(*steps, fmt.Sprintf("%s: %s to (%.0f, %.0f, %.0f) rotation %.0f",
		desc, tcpFrame, p.X, p.Y, p.Z, rotationDeg))
	return nil
}

// runPick runs pick_snack. It tries the plausible bags best-first, each at
// every gripper rotation, until one can be reached. dryRun stops after
// finding the bags; hoverOnly stops above the first bag it can hover over.
func (s *snackRestockerRestocker) runPick(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if s.vision == nil || s.motion == nil {
		return nil, fmt.Errorf("pick_snack needs the '%s' vision service and '%s' motion service on the machine",
			sam3VisionName, motionName)
	}
	dryRun, _ := cmd["dry_run"].(bool)
	hoverOnly, _ := cmd["hover_only"].(bool)
	approachMm := defaultApproachMm
	if v, ok := cmd["approach_mm"].(float64); ok {
		approachMm = v
	}
	graspDepthMm := defaultGraspDepthMm
	if v, ok := cmd["grasp_depth_mm"].(float64); ok {
		graspDepthMm = v
	}
	maxBags := defaultMaxBags
	if v, ok := cmd["max_bags"].(float64); ok && v >= 1 {
		maxBags = int(v)
	}

	steps := []interface{}{}
	step := 0
	next := func() int {
		step++
		return step
	}

	// Look at the bin from the first restock pose.
	if err := s.goToPose(ctx, pickViewSwitch, next(), &steps); err != nil {
		return nil, err
	}
	targets, total, err := s.findSnacks(ctx)
	if err != nil {
		return nil, err
	}
	rejected := total - len(targets)
	if len(targets) > maxBags {
		targets = targets[:maxBags]
	}

	attempts := []interface{}{}
	result := map[string]interface{}{
		"candidates": total,
		"rejected":   rejected,
		"plausible":  len(targets),
		"dry_run":    dryRun,
		"hover_only": hoverOnly,
	}
	// selectBag records the bag (1 = most confident plausible bag) and
	// rotation used in the result.
	selectBag := func(rank int, t *snackTarget, rotation float64, hover, grasp r3.Vector) {
		result["bag"] = rank
		result["label"] = t.label
		result["confidence"] = t.confidence
		result["center_mm"] = vec(t.center)
		result["top_z_mm"] = t.topZ
		result["rotation_deg"] = rotation
		result["hover_mm"] = vec(hover)
		result["grasp_mm"] = vec(grasp)
	}
	finish := func(status string) map[string]interface{} {
		result["pick_snack"] = status
		result["attempts"] = attempts
		result["steps"] = steps
		result["steps_completed"] = step
		return result
	}
	posesFor := func(t *snackTarget) (r3.Vector, r3.Vector) {
		return r3.Vector{X: t.center.X, Y: t.center.Y, Z: t.topZ + approachMm},
			r3.Vector{X: t.center.X, Y: t.center.Y, Z: t.topZ - graspDepthMm}
	}

	if dryRun {
		hover, grasp := posesFor(targets[0])
		selectBag(1, targets[0], gripperRotations[0], hover, grasp)
		return finish("planned"), nil
	}

	if err := s.setGripper(ctx, gripperOpen, next(), &steps); err != nil {
		return nil, err
	}
	for i, t := range targets {
		hover, grasp := posesFor(t)
		s.logger.Infof("pick: trying bag %d of %d, %q (confidence %.2f), center (%.0f, %.0f, %.0f) mm, top z %.0f mm",
			i+1, len(targets), t.label, t.confidence, t.center.X, t.center.Y, t.center.Z, t.topZ)
		for _, rotation := range gripperRotations {
			// A refused hover or descend is a planning failure: record it and
			// try the next rotation or bag. The arm is left at the last pose
			// it reached, and the next move plans from there.
			refused := func(stage string, err error) {
				s.logger.Infof("pick: bag %d rotation %.0f: %s refused: %v", i+1, rotation, stage, err)
				attempts = append(attempts, map[string]interface{}{
					"bag":          i + 1,
					"confidence":   t.confidence,
					"center_mm":    vec(t.center),
					"rotation_deg": rotation,
					"stage":        stage,
					"error":        err.Error(),
				})
			}
			if err := s.moveTCP(ctx, "hover above snack", hover, rotation, nil, next(), &steps); err != nil {
				if ctx.Err() != nil {
					return nil, err
				}
				refused("hover", err)
				continue
			}
			if hoverOnly {
				selectBag(i+1, t, rotation, hover, grasp)
				return finish("hovering"), nil
			}
			if err := s.moveTCP(ctx, "descend to snack", grasp, rotation, straightDown, next(), &steps); err != nil {
				if ctx.Err() != nil {
					return nil, err
				}
				refused("descend", err)
				continue
			}
			// At the bag: from here a failure is not retried.
			selectBag(i+1, t, rotation, hover, grasp)
			if err := s.setGripper(ctx, gripperClose, next(), &steps); err != nil {
				return nil, err
			}
			if err := s.moveTCP(ctx, "lift snack", hover, rotation, straightDown, next(), &steps); err != nil {
				return nil, err
			}
			return finish("complete"), nil
		}
	}
	result["error"] = fmt.Sprintf("could not reach any of the %d plausible bags at rotations %v; see attempts",
		len(targets), gripperRotations)
	return finish("failed"), nil
}

// Poses pick_and_place carries a picked bag through before releasing it over
// the drawer: the same drop poses the restock sequence uses.
var placePoses = []string{"restock-4-view-drawer", "restock-5-drop"}

// How many pick-then-place rounds pick_and_place runs, by default.
const defaultPlaceRounds = 2

// runPickPlace runs pick_and_place: rounds of pick_snack, each followed by
// moving the bag through placePoses and opening the gripper to drop it. It
// takes the same options as pick_snack plus "rounds". It stops at the first
// round whose pick does not complete, including a dry_run or hover_only pick.
func (s *snackRestockerRestocker) runPickPlace(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	rounds := defaultPlaceRounds
	if v, ok := cmd["rounds"].(float64); ok && v >= 1 {
		rounds = int(v)
	}
	results := []interface{}{}
	completed := 0
	finish := func(status string) map[string]interface{} {
		return map[string]interface{}{
			"pick_and_place":   status,
			"rounds":           rounds,
			"rounds_completed": completed,
			"results":          results,
		}
	}

	for r := 1; r <= rounds; r++ {
		pick, err := s.runPick(ctx, cmd)
		if err != nil {
			return nil, fmt.Errorf("round %d pick: %w", r, err)
		}
		round := map[string]interface{}{"round": r, "pick": pick}
		results = append(results, round)
		if status, _ := pick["pick_snack"].(string); status != "complete" {
			result := finish(status)
			if status == "failed" {
				result["error"] = fmt.Sprintf("round %d pick failed: %v", r, pick["error"])
			}
			return result, nil
		}

		steps := []interface{}{}
		step := 0
		next := func() int {
			step++
			return step
		}
		for _, name := range placePoses {
			if err := s.goToPose(ctx, name, next(), &steps); err != nil {
				return nil, fmt.Errorf("round %d place: %w", r, err)
			}
		}
		if err := s.setGripper(ctx, gripperOpen, next(), &steps); err != nil {
			return nil, fmt.Errorf("round %d place: %w", r, err)
		}
		round["place_steps"] = steps
		completed++
	}
	return finish("complete"), nil
}
