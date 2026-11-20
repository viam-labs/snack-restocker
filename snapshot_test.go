package snackrestocker

import (
	"bytes"
	"context"
	"encoding/base64"
	"errors"
	"fmt"
	"path/filepath"
	"reflect"
	"strings"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/vision/detection3d"
)

type uploaded struct {
	data   []byte
	name   string
	tags   []string
	params map[string]interface{}
}

type snapshotRig struct {
	*pickRig
	renders []map[string]interface{}
	uploads []uploaded
}

var fakeJPEG = []byte{0xff, 0xd8, 0xff, 0xd9}

// newSnapshotRig is a pick rig whose sam3-vision renders snapshots, and whose
// uploads are recorded instead of sent.
func newSnapshotRig(t *testing.T, dets []*detection3d.Detection, uploadErr error) *snapshotRig {
	rig := &snapshotRig{pickRig: newPickRig(t, dets)}
	v := rig.s.vision.(*inject.VisionService)
	v.DoCommandFunc = func(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
		if cmd["command"] != "render_pick" {
			t.Fatalf("unexpected sam3 command: %v", cmd)
		}
		rig.renders = append(rig.renders, cmd)
		return map[string]interface{}{"image_jpeg_b64": base64.StdEncoding.EncodeToString(fakeJPEG)}, nil
	}
	rig.s.uploadData = func(
		ctx context.Context, data []byte, name string, tags []string, params map[string]interface{},
	) (string, error) {
		rig.uploads = append(rig.uploads, uploaded{data: data, name: name, tags: tags, params: params})
		if uploadErr != nil {
			return "", uploadErr
		}
		return fmt.Sprintf("binary-%d", len(rig.uploads)), nil
	}
	return rig
}

func (r *snapshotRig) byExt(ext string) []uploaded {
	var out []uploaded
	for _, u := range r.uploads {
		if filepath.Ext(u.name) == ext {
			out = append(out, u)
		}
	}
	return out
}

func (r *snapshotRig) imageTags() [][]string {
	var out [][]string
	for _, u := range r.byExt(".jpg") {
		out = append(out, u.tags)
	}
	return out
}

// cloudOutcomes maps each uploaded cloud's segment index to its outcome.
func (r *snapshotRig) cloudOutcomes() map[float64]string {
	out := map[float64]string{}
	for _, u := range r.byExt(".pcd") {
		out[u.params["segment"].(float64)] = u.params["outcome"].(string)
	}
	return out
}

func (r *snapshotRig) onlyRender(t *testing.T) map[string]interface{} {
	t.Helper()
	if len(r.renders) != 1 || len(r.snapshotIDs) != 1 {
		t.Fatalf("want one scan and one render_pick, got scans %v renders %v", r.snapshotIDs, r.renders)
	}
	if id := r.snapshotIDs[0]; id == "" || r.renders[0]["snapshot_id"] != id {
		t.Fatalf("render_pick snapshot %v, want the scan's %q", r.renders[0]["snapshot_id"], id)
	}
	return r.renders[0]
}

func TestPickSnapshotHighlightsTheBagActuallyGrabbed(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(t), nil)
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return p.Z == 80 } // bag 1 unreachable
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
	if !reflect.DeepEqual(rig.imageTags(), [][]string{{"pick_snack", "pick_snack_complete"}}) {
		t.Fatalf("image uploads: %v", rig.uploads)
	}
	if image := rig.byExt(".jpg")[0]; string(image.data) != string(fakeJPEG) {
		t.Fatalf("uploaded image is not the rendered one")
	}
	if resp["snapshot_binary_data_id"] != "binary-1" {
		t.Fatalf("result should name the image upload: %v", resp)
	}
}

func TestPickCloudsLabelEveryTriedBag(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(t), nil)
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return p.Z == 80 } // bag 1 unreachable
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" || resp["bag"] != 2 {
		t.Fatalf("pick: %v", resp)
	}
	if got := rig.cloudOutcomes(); !reflect.DeepEqual(got, map[float64]string{0: "unreachable", 1: "complete"}) {
		t.Fatalf("cloud outcomes: %v", got)
	}
	snapshotID := rig.snapshotIDs[0]
	for _, u := range rig.byExt(".pcd") {
		outcome := u.params["outcome"].(string)
		if !reflect.DeepEqual(u.tags, []string{"pick_snack_cloud", "pick_snack_cloud_" + outcome}) {
			t.Fatalf("cloud tags: %v", u.tags)
		}
		if u.params["snapshot_id"] != snapshotID || u.params["image_binary_data_id"] != "binary-1" ||
			u.params["pick_status"] != "complete" {
			t.Fatalf("cloud should link to its pick and image: %v", u.params)
		}
		if !strings.HasPrefix(u.name, snapshotID+"_seg") || !strings.HasSuffix(u.name, "_"+outcome+".pcd") {
			t.Fatalf("cloud file name: %q", u.name)
		}
	}
	ids := resp["pointcloud_binary_data_ids"].([]interface{})
	if len(ids) != 2 || resp["pointcloud_error"] != nil {
		t.Fatalf("result should name both cloud uploads: %v", resp)
	}
}

// The saved cloud is the grabbed bag's points in the world frame, not
// relative to the detection's root frame.
func TestPickCloudIsTheBagInTheWorldFrame(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(t), nil)
	rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	clouds := rig.byExt(".pcd")
	if len(clouds) != 1 || clouds[0].params["outcome"] != "complete" {
		t.Fatalf("want only the grabbed bag's cloud: %v", rig.uploads)
	}
	pc, err := pointcloud.ReadPCD(bytes.NewReader(clouds[0].data), "")
	if err != nil {
		t.Fatal(err)
	}
	// Bag 1: center (250, -440, 70), 100 x 80 x 60 mm.
	md := pc.MetaData()
	near := func(a, b float64) bool { return a-b < 1e-3 && b-a < 1e-3 }
	if pc.Size() != 8 || !near(md.MinX, 200) || !near(md.MaxX, 300) || !near(md.MinY, -480) ||
		!near(md.MaxY, -400) || !near(md.MinZ, 40) || !near(md.MaxZ, 100) {
		t.Fatalf("cloud bounds %+v (size %d), want the bag's world bounds", md, pc.Size())
	}
}

func TestPickSnapshotDryRun(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(t), nil)
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "dry_run": true})
	if resp["pick_snack"] != "planned" {
		t.Fatalf("dry run: %v", resp)
	}
	render := rig.onlyRender(t)
	if render["target"] != 0.0 {
		t.Fatalf("dry run should highlight the planned bag: %v", render)
	}
	if !reflect.DeepEqual(rig.imageTags(), [][]string{{"pick_snack", "pick_snack_planned"}}) {
		t.Fatalf("image uploads: %v", rig.uploads)
	}
	if clouds := rig.byExt(".pcd"); len(clouds) != 0 {
		t.Fatalf("a dry run tries no grasp, so it saves no clouds: %v", clouds)
	}
}

func TestPickCloudsNotSavedWhenOnlyHovering(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(t), nil)
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return a.rotationDeg == 0 }
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "hover_only": true})
	if resp["pick_snack"] != "hovering" {
		t.Fatalf("hover: %v", resp)
	}
	if clouds := rig.byExt(".pcd"); len(clouds) != 0 {
		t.Fatalf("hover_only tries no grasp, so it saves no clouds: %v", clouds)
	}
}

func TestPickSnapshotWhenNoSegmentIsPlausible(t *testing.T) {
	outsideBin := segment(t, 0.93, r3.Vector{X: 357, Y: -1539, Z: -887}, r3.Vector{X: 360, Y: 245, Z: 152})
	fragment := segment(t, 0.92, r3.Vector{X: 399, Y: -297, Z: 57}, r3.Vector{X: 6, Y: 7, Z: 4})
	rig := newSnapshotRig(t, []*detection3d.Detection{outsideBin, fragment}, nil)
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
	if !reflect.DeepEqual(rig.imageTags(), [][]string{{"pick_snack", "pick_snack_failed"}}) {
		t.Fatalf("image uploads: %v", rig.uploads)
	}
	if clouds := rig.byExt(".pcd"); len(clouds) != 0 {
		t.Fatalf("no bag was tried: %v", clouds)
	}
}

// With no detections there is no metadata to carry a snapshot ID, so the
// render relies on the ID pick_snack chose for the scan.
func TestPickSnapshotWhenNothingIsFound(t *testing.T) {
	rig := newSnapshotRig(t, nil, nil)
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "failed" {
		t.Fatalf("pick: %v", resp)
	}
	rig.onlyRender(t)
	if !reflect.DeepEqual(rig.imageTags(), [][]string{{"pick_snack", "pick_snack_failed"}}) {
		t.Fatalf("image uploads: %v", rig.uploads)
	}
}

func TestPickSnapshotMarksBagsBeyondMaxBags(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(t), nil)
	rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "dry_run": true, "max_bags": 1.0})
	notes := rig.onlyRender(t)["notes"].(map[string]interface{})
	if notes["0"] != "TARGET bag 1" || notes["1"] != "not tried (max_bags)" {
		t.Fatalf("notes: %v", notes)
	}
}

func TestPickSnapshotUploadFailureDoesNotFailThePick(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(t), errors.New("permission denied"))
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" {
		t.Fatalf("pick: %v", resp)
	}
	if resp["snapshot_error"] != "upload: permission denied" || resp["snapshot_binary_data_id"] != nil {
		t.Fatalf("snapshot result: %v", resp)
	}
	if resp["pointcloud_error"] != "segment 0: upload: permission denied" || resp["pointcloud_binary_data_ids"] != nil {
		t.Fatalf("point cloud result: %v", resp)
	}
	if cloud := rig.byExt(".pcd")[0]; cloud.params["image_binary_data_id"] != nil {
		t.Fatalf("a cloud should not link to an image that failed to upload: %v", cloud.params)
	}
}

func TestPickSnapshotOnEveryPickAndPlaceRound(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(t), nil)
	withPlacePoses(rig.pickRig)
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_and_place": true})
	if resp["pick_and_place"] != "complete" {
		t.Fatalf("pick_and_place: %v", resp)
	}
	if n := len(rig.imageTags()); n != defaultPlaceRounds {
		t.Fatalf("want a snapshot per round, got %v", rig.uploads)
	}
	if n := len(rig.byExt(".pcd")); n != defaultPlaceRounds {
		t.Fatalf("want the grabbed bag's cloud per round, got %v", rig.uploads)
	}
}

func TestPickSnapshotBagGrabbedAfterARefusedRotationIsNotFailed(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(t), nil)
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return p.Z == 80 && a.rotationDeg == 0 }
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
	if got := rig.cloudOutcomes(); !reflect.DeepEqual(got, map[float64]string{0: "complete"}) {
		t.Fatalf("bag 1 was grabbed, so its cloud is complete, not unreachable: %v", got)
	}
}

func TestPickSnapshotAllTriedBagsFailed(t *testing.T) {
	rig := newSnapshotRig(t, twoBags(t), nil)
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return p.Z > 150 } // every hover
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
	if got := rig.cloudOutcomes(); !reflect.DeepEqual(got, map[float64]string{0: "unreachable", 1: "unreachable"}) {
		t.Fatalf("cloud outcomes: %v", got)
	}
}
