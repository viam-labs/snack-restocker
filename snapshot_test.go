package snackrestocker

import (
	"context"
	"encoding/base64"
	"errors"
	"reflect"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/testutils/inject"
)

type snapshotRig struct {
	*pickRig
	renders []map[string]interface{}
	uploads [][]string
	jpegs   [][]byte
}

var fakeJPEG = []byte{0xff, 0xd8, 0xff, 0xd9}

// newSnapshotRig is a pick rig whose sam3-vision returns a snapshot ID and
// renders it, and whose uploads are recorded instead of sent.
func newSnapshotRig(t *testing.T, sam3Resp map[string]interface{}, uploadErr error) *snapshotRig {
	sam3Resp["snapshot_id"] = "snap-1"
	rig := &snapshotRig{pickRig: newPickRig(t, sam3Resp)}
	v := inject.NewVisionService(sam3VisionName)
	v.DoCommandFunc = func(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
		switch cmd["command"] {
		case "get_object_point_clouds":
			return sam3Resp, nil
		case "render_pick":
			rig.renders = append(rig.renders, cmd)
			return map[string]interface{}{"image_jpeg_b64": base64.StdEncoding.EncodeToString(fakeJPEG)}, nil
		}
		t.Fatalf("unexpected sam3 command: %v", cmd)
		return nil, nil
	}
	rig.s.vision = v
	rig.s.uploadSnapshot = func(ctx context.Context, jpeg []byte, tags []string) (string, error) {
		rig.uploads = append(rig.uploads, tags)
		rig.jpegs = append(rig.jpegs, jpeg)
		if uploadErr != nil {
			return "", uploadErr
		}
		return "binary-1", nil
	}
	return rig
}

func (r *snapshotRig) onlyRender(t *testing.T) map[string]interface{} {
	t.Helper()
	if len(r.renders) != 1 {
		t.Fatalf("want one render_pick, got %v", r.renders)
	}
	if r.renders[0]["snapshot_id"] != "snap-1" {
		t.Fatalf("render_pick snapshot: %v", r.renders[0])
	}
	return r.renders[0]
}

func TestPickSnapshotHighlightsTheBagActuallyGrabbed(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(), nil)
	rig.refuse = func(p r3.Vector, rotation float64) bool { return p.Z == 80 } // bag 1 unreachable
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" || resp["bag"] != 2 {
		t.Fatalf("pick: %v", resp)
	}
	render := rig.onlyRender(t)
	if render["target"] != 1.0 {
		t.Fatalf("should highlight segment 1 (bag 2): %v", render)
	}
	wantNotes := map[string]interface{}{"0": "bag 1: descend refused", "1": "TARGET bag 2"}
	if !reflect.DeepEqual(render["notes"], wantNotes) {
		t.Fatalf("notes: %v", render["notes"])
	}
	if !reflect.DeepEqual(render["failed"], []interface{}{0.0}) {
		t.Fatalf("bag 1 should be drawn as a failed try: %v", render["failed"])
	}
	if render["min_confidence"] != snapshotMinConfidence {
		t.Fatalf("min_confidence: %v", render["min_confidence"])
	}
	if !reflect.DeepEqual(rig.uploads, [][]string{{"pick_snack", "pick_snack_complete"}}) {
		t.Fatalf("uploads: %v", rig.uploads)
	}
	if string(rig.jpegs[0]) != string(fakeJPEG) {
		t.Fatalf("uploaded image is not the rendered one")
	}
	if resp["snapshot_binary_data_id"] != "binary-1" {
		t.Fatalf("result should name the upload: %v", resp)
	}
}

func TestPickSnapshotDryRun(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(), nil)
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "dry_run": true})
	if resp["pick_snack"] != "planned" {
		t.Fatalf("dry run: %v", resp)
	}
	render := rig.onlyRender(t)
	if render["target"] != 0.0 {
		t.Fatalf("dry run should highlight the planned bag: %v", render)
	}
	if !reflect.DeepEqual(rig.uploads, [][]string{{"pick_snack", "pick_snack_planned"}}) {
		t.Fatalf("uploads: %v", rig.uploads)
	}
}

func TestPickSnapshotWhenNoSegmentIsPlausible(t *testing.T) {
	outsideBin := segment(0.93, r3.Vector{X: 357, Y: -1539, Z: -887}, r3.Vector{X: 360, Y: 245, Z: 152})
	fragment := segment(0.92, r3.Vector{X: 399, Y: -297, Z: 57}, r3.Vector{X: 6, Y: 7, Z: 4})
	rig := newSnapshotRig(t, map[string]interface{}{"objects": []interface{}{outsideBin, fragment}}, nil)
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "failed" {
		t.Fatalf("pick: %v", resp)
	}
	render := rig.onlyRender(t)
	if render["target"] != -1.0 {
		t.Fatalf("nothing should be highlighted: %v", render)
	}
	wantNotes := map[string]interface{}{"0": "outside bin", "1": "not bag-sized"}
	if !reflect.DeepEqual(render["notes"], wantNotes) {
		t.Fatalf("notes: %v", render["notes"])
	}
	if !reflect.DeepEqual(rig.uploads, [][]string{{"pick_snack", "pick_snack_failed"}}) {
		t.Fatalf("uploads: %v", rig.uploads)
	}
}

func TestPickSnapshotMarksBagsBeyondMaxBags(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(), nil)
	rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "dry_run": true, "max_bags": 1.0})
	notes := rig.onlyRender(t)["notes"].(map[string]interface{})
	if notes["0"] != "TARGET bag 1" || notes["1"] != "not tried (max_bags)" {
		t.Fatalf("notes: %v", notes)
	}
}

func TestPickSnapshotUploadFailureDoesNotFailThePick(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(), errors.New("permission denied"))
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" {
		t.Fatalf("pick: %v", resp)
	}
	if resp["snapshot_error"] != "upload: permission denied" || resp["snapshot_binary_data_id"] != nil {
		t.Fatalf("snapshot result: %v", resp)
	}
}

func TestPickSnapshotOnEveryPickAndPlaceRound(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(), nil)
	withPlacePoses(rig.pickRig)
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_and_place": true})
	if resp["pick_and_place"] != "complete" {
		t.Fatalf("pick_and_place: %v", resp)
	}
	if len(rig.uploads) != defaultPlaceRounds {
		t.Fatalf("want a snapshot per round, got %v", rig.uploads)
	}
}

func TestPickSnapshotBagGrabbedAfterARefusedRotationIsNotFailed(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(), nil)
	rig.refuse = func(p r3.Vector, rotation float64) bool { return p.Z == 80 && rotation == 0 }
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" || resp["bag"] != 1 {
		t.Fatalf("pick: %v", resp)
	}
	render := rig.onlyRender(t)
	if render["target"] != 0.0 || len(render["failed"].([]interface{})) != 0 {
		t.Fatalf("bag 1 was grabbed, so it is the target, not a failure: %v", render)
	}
	if render["notes"].(map[string]interface{})["0"] != "TARGET bag 1" {
		t.Fatalf("notes: %v", render["notes"])
	}
}

func TestPickSnapshotAllTriedBagsFailed(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(), nil)
	rig.refuse = func(p r3.Vector, rotation float64) bool { return p.Z > 150 } // every hover
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "failed" {
		t.Fatalf("pick: %v", resp)
	}
	render := rig.onlyRender(t)
	failed := render["failed"].([]interface{})
	if len(failed) != 2 || render["target"] != -1.0 {
		t.Fatalf("both bags should be failed tries with no target: %v", render)
	}
	notes := render["notes"].(map[string]interface{})
	if notes["0"] != "bag 1: hover refused" || notes["1"] != "bag 2: hover refused" {
		t.Fatalf("notes: %v", notes)
	}
}
