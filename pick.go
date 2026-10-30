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

// Straight-line tolerance for the descend and lift moves.
var straightDown = &motionplan.Constraints{
	LinearConstraint: []motionplan.LinearConstraint{{LineToleranceMm: 10, OrientationToleranceDegs: 5}},
}

// snackTarget is the best SAM3 segment, in the world frame.
type snackTarget struct {
	label      string
	confidence float64
	center     r3.Vector
	topZ       float64
	candidates int
	rejected   int
}

// topDown is the tool pointing straight down (-Z in world).
func topDown(p r3.Vector) *referenceframe.PoseInFrame {
	return referenceframe.NewPoseInFrame(referenceframe.World,
		spatialmath.NewPose(p, &spatialmath.OrientationVectorDegrees{OZ: -1}))
}

func vec(v r3.Vector) map[string]interface{} {
	return map[string]interface{}{"x": v.X, "y": v.Y, "z": v.Z}
}

// findSnack asks sam3-vision for segments (sorted best-first) and returns the
// first one that could be a real bag in the bin.
func (s *snackRestockerRestocker) findSnack(ctx context.Context) (*snackTarget, error) {
	resp, err := s.vision.DoCommand(ctx, map[string]interface{}{"command": "get_object_point_clouds"})
	if err != nil {
		return nil, fmt.Errorf("%s get_object_point_clouds: %w", sam3VisionName, err)
	}
	if msg, ok := resp["error"]; ok {
		return nil, fmt.Errorf("%s: %v", sam3VisionName, msg)
	}
	objects, _ := resp["objects"].([]interface{})
	if len(objects) == 0 {
		return nil, errors.New("no snack bags found")
	}
	var reasons []string
	for i, raw := range objects {
		target, reason := parseSnack(raw)
		if target == nil {
			s.logger.Debugf("pick: skipping segment %d: %s", i, reason)
			reasons = append(reasons, reason)
			continue
		}
		target.candidates = len(objects)
		target.rejected = len(reasons)
		return target, nil
	}
	return nil, fmt.Errorf("none of the %d segments is a plausible bag in the bin; first rejection: %s",
		len(objects), reasons[0])
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

// moveTCP plans and runs a move of the gripper TCP to a world pose.
func (s *snackRestockerRestocker) moveTCP(
	ctx context.Context, desc string, p r3.Vector, constraints *motionplan.Constraints, step int, steps *[]interface{},
) error {
	s.logger.Infof("pick step %d: %s, moving %s to (%.0f, %.0f, %.0f) mm", step, desc, tcpFrame, p.X, p.Y, p.Z)
	if _, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: tcpFrame,
		Destination:   topDown(p),
		Constraints:   constraints,
	}); err != nil {
		return fmt.Errorf("%s: %w", desc, err)
	}
	if err := sleep(ctx, stepDelay); err != nil {
		return err
	}
	*steps = append(*steps, fmt.Sprintf("%s: %s to (%.0f, %.0f, %.0f)", desc, tcpFrame, p.X, p.Y, p.Z))
	return nil
}

// runPick runs pick_snack. dryRun stops after finding the bag; hoverOnly
// stops above it with the gripper open.
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
	target, err := s.findSnack(ctx)
	if err != nil {
		return nil, err
	}
	hover := r3.Vector{X: target.center.X, Y: target.center.Y, Z: target.topZ + approachMm}
	grasp := r3.Vector{X: target.center.X, Y: target.center.Y, Z: target.topZ - graspDepthMm}
	s.logger.Infof("pick: best of %d is %q (confidence %.2f), center (%.0f, %.0f, %.0f) mm, top z %.0f mm",
		target.candidates, target.label, target.confidence,
		target.center.X, target.center.Y, target.center.Z, target.topZ)

	result := map[string]interface{}{
		"label":      target.label,
		"confidence": target.confidence,
		"candidates": target.candidates,
		"rejected":   target.rejected,
		"center_mm":  vec(target.center),
		"top_z_mm":   target.topZ,
		"hover_mm":   vec(hover),
		"grasp_mm":   vec(grasp),
		"dry_run":    dryRun,
		"hover_only": hoverOnly,
	}
	finish := func(status string) map[string]interface{} {
		result["pick_snack"] = status
		result["steps"] = steps
		result["steps_completed"] = step
		return result
	}
	if dryRun {
		return finish("planned"), nil
	}

	if err := s.setGripper(ctx, gripperOpen, next(), &steps); err != nil {
		return nil, err
	}
	if err := s.moveTCP(ctx, "hover above snack", hover, nil, next(), &steps); err != nil {
		return nil, err
	}
	if hoverOnly {
		return finish("hovering"), nil
	}
	if err := s.moveTCP(ctx, "descend to snack", grasp, straightDown, next(), &steps); err != nil {
		return nil, err
	}
	if err := s.setGripper(ctx, gripperClose, next(), &steps); err != nil {
		return nil, err
	}
	if err := s.moveTCP(ctx, "lift snack", hover, straightDown, next(), &steps); err != nil {
		return nil, err
	}
	return finish("complete"), nil
}
