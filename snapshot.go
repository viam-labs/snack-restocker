package snackrestocker

import (
	"bytes"
	"context"
	"encoding/base64"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"go.viam.com/rdk/app"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
)

// Per-pick records: after each pick_snack run, sam3-vision draws the frame it
// segmented with every candidate labeled and the chosen bag highlighted, and
// the image plus the point cloud of every bag the arm tried are uploaded to
// Viam data so each pick can be reviewed and used as training data.

const (
	snapshotMethod  = "pick_snack"
	snapshotTag     = "pick_snack"
	cloudTag        = "pick_snack_cloud"
	snapshotTimeout = 20 * time.Second
	// Segments below this SAM3 confidence are left off the snapshot unless the pick tried them.
	snapshotMinConfidence = 0.75
	// Outcome of a bag whose every hover or descend was refused.
	outcomeUnreachable = "unreachable"
)

// dataUploader stores one file and returns its binary data ID. The file name's
// extension sets the data type.
type dataUploader func(ctx context.Context, data []byte, fileName string, tags []string,
	params map[string]interface{}) (string, error)

// shortReason turns a parseSnack rejection into a few words for the image.
func shortReason(reason string) string {
	switch {
	case strings.Contains(reason, "outside the bin"):
		return "outside bin"
	case strings.Contains(reason, "not bag-sized"):
		return "not bag-sized"
	case strings.Contains(reason, "not world"):
		return "not in world frame"
	default:
		return "rejected"
	}
}

// recordPick uploads the snapshot and point clouds for one pick_snack run. It
// never fails the pick: problems are logged and reported in result as
// snapshot_error and pointcloud_error.
func (s *snackRestockerRestocker) recordPick(
	ctx context.Context, scan *sam3Scan, chosen *snackTarget, status string, result map[string]interface{},
) {
	if scan == nil {
		return
	}
	set := func(k string, v interface{}) {
		if result != nil {
			result[k] = v
		}
		s.statusEvent("", "", map[string]interface{}{k: v})
	}
	ctx, cancel := context.WithTimeout(context.WithoutCancel(ctx), snapshotTimeout)
	defer cancel()
	upload := s.uploadData
	if upload == nil {
		upload = s.uploadToData
	}
	imageID := s.recordSnapshot(ctx, scan, chosen, status, upload, set)
	s.recordClouds(ctx, scan, chosen, status, imageID, upload, set)
}

// recordSnapshot uploads the rendered pick image and returns its binary data
// ID, or "" if it could not.
func (s *snackRestockerRestocker) recordSnapshot(
	ctx context.Context, scan *sam3Scan, chosen *snackTarget, status string,
	upload dataUploader, set func(string, interface{}),
) string {
	target := -1.0
	title := "pick_snack " + status
	if chosen != nil {
		target = float64(chosen.index)
		title += fmt.Sprintf(" - bag #%d (%.2f)", chosen.index+1, chosen.confidence)
	}
	notes := map[string]interface{}{}
	for i, note := range scan.notes {
		notes[strconv.Itoa(i)] = note
	}
	failed := []interface{}{}
	for i := range scan.failed {
		if chosen == nil || i != chosen.index {
			failed = append(failed, float64(i))
		}
	}
	resp, err := s.vision.DoCommand(ctx, map[string]interface{}{
		"command":        "render_pick",
		"snapshot_id":    scan.snapshotID,
		"target":         target,
		"failed":         failed,
		"min_confidence": snapshotMinConfidence,
		"notes":          notes,
		"title":          title,
	})
	if err == nil {
		if msg, ok := resp["error"]; ok {
			err = fmt.Errorf("%v", msg)
		}
	}
	var jpeg []byte
	if err == nil {
		encoded, _ := resp["image_jpeg_b64"].(string)
		jpeg, err = base64.StdEncoding.DecodeString(encoded)
		if err == nil && len(jpeg) == 0 {
			err = errors.New("empty image")
		}
	}
	if err != nil {
		s.logger.Warnf("pick snapshot: %s render_pick: %v", sam3VisionName, err)
		set("snapshot_error", "render_pick: "+err.Error())
		return ""
	}

	id, err := upload(ctx, jpeg, scan.snapshotID+".jpg", []string{snapshotTag, snapshotTag + "_" + status},
		map[string]interface{}{"snapshot_id": scan.snapshotID, "status": status})
	if err != nil {
		s.logger.Warnf("pick snapshot: upload: %v", err)
		set("snapshot_error", "upload: "+err.Error())
		return ""
	}
	s.logger.Infof("pick snapshot uploaded: binary data id %s (tags %s, %s_%s)", id, snapshotTag, snapshotTag, status)
	set("snapshot_binary_data_id", id)
	return id
}

// cloudOutcome is how a bag's grasp ended: the pick status for the bag the arm
// reached, unreachable for one it could not, and "" for one it never tried or
// a run that never tried to grasp.
func cloudOutcome(scan *sam3Scan, t, chosen *snackTarget, status string) string {
	switch {
	case status == "planned" || status == "hovering":
		return ""
	case t == chosen:
		return status
	case scan.failed[t.index]:
		return outcomeUnreachable
	default:
		return ""
	}
}

// recordClouds uploads the world-frame point cloud of every bag the arm tried,
// tagged with how the try ended and linked to the pick image.
func (s *snackRestockerRestocker) recordClouds(
	ctx context.Context, scan *sam3Scan, chosen *snackTarget, status, imageID string,
	upload dataUploader, set func(string, interface{}),
) {
	ids := []interface{}{}
	var errs []string
	for _, t := range scan.targets {
		outcome := cloudOutcome(scan, t, chosen, status)
		if outcome == "" {
			continue
		}
		var buf bytes.Buffer
		if err := pointcloud.ToPCD(t.cloud, &buf, pointcloud.PCDBinary); err != nil {
			errs = append(errs, fmt.Sprintf("segment %d: encode: %v", t.index, err))
			continue
		}
		params := map[string]interface{}{
			"outcome":         outcome,
			"pick_status":     status,
			"snapshot_id":     scan.snapshotID,
			"segment":         float64(t.index),
			"label":           t.label,
			"confidence":      t.confidence,
			"reference_frame": referenceframe.World,
			"center_mm":       vec(t.center),
			"top_z_mm":        t.topZ,
		}
		if imageID != "" {
			params["image_binary_data_id"] = imageID
		}
		name := fmt.Sprintf("%s_seg%d_%s.pcd", scan.snapshotID, t.index, outcome)
		id, err := upload(ctx, buf.Bytes(), name, []string{cloudTag, cloudTag + "_" + outcome}, params)
		if err != nil {
			errs = append(errs, fmt.Sprintf("segment %d: upload: %v", t.index, err))
			continue
		}
		ids = append(ids, id)
	}
	if len(errs) > 0 {
		s.logger.Warnf("pick point clouds: %s", strings.Join(errs, "; "))
		set("pointcloud_error", strings.Join(errs, "; "))
	}
	if len(ids) > 0 {
		s.logger.Infof("pick point clouds uploaded: binary data ids %v", ids)
		set("pointcloud_binary_data_ids", ids)
	}
}

// uploadToData uploads to Viam data as this service's pick_snack capture,
// using the API key viam-server gives the module.
func (s *snackRestockerRestocker) uploadToData(
	ctx context.Context, data []byte, fileName string, tags []string, params map[string]interface{},
) (string, error) {
	partID := os.Getenv("VIAM_MACHINE_PART_ID")
	if partID == "" {
		return "", errors.New("VIAM_MACHINE_PART_ID is not set")
	}
	client, err := s.dataClient(ctx)
	if err != nil {
		return "", err
	}
	now := time.Now()
	return client.DataClient().BinaryDataCaptureUpload(
		ctx, data, partID, "rdk:service:generic", s.name.ShortName(), snapshotMethod, filepath.Ext(fileName),
		&app.BinaryDataCaptureUploadOptions{
			FileName: &fileName, Tags: tags, MethodParameters: params, DataRequestTimes: &[2]time.Time{now, now},
		},
	)
}

func (s *snackRestockerRestocker) dataClient(ctx context.Context) (*app.ViamClient, error) {
	s.viamClientMu.Lock()
	defer s.viamClientMu.Unlock()
	if s.viamClient != nil {
		return s.viamClient, nil
	}
	client, err := app.CreateViamClientFromEnvVars(ctx, nil, s.logger)
	if err != nil {
		return nil, fmt.Errorf("connect to Viam app: %w", err)
	}
	s.viamClient = client
	return client, nil
}

func (s *snackRestockerRestocker) closeDataClient() {
	s.viamClientMu.Lock()
	defer s.viamClientMu.Unlock()
	if s.viamClient != nil {
		if err := s.viamClient.Close(); err != nil {
			s.logger.Debugf("closing Viam app client: %v", err)
		}
		s.viamClient = nil
	}
}
