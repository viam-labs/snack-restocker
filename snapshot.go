package snackrestocker

import (
	"context"
	"encoding/base64"
	"errors"
	"fmt"
	"os"
	"strconv"
	"strings"
	"time"

	"go.viam.com/rdk/app"
)

// Per-pick snapshot: after each pick_snack, sam3-vision draws the frame it
// segmented with every candidate labeled and the chosen bag highlighted, and
// the image is uploaded to Viam data so each pick can be reviewed later.

const (
	snapshotMethod  = "pick_snack"
	snapshotTag     = "pick_snack"
	snapshotTimeout = 20 * time.Second
	// Segments below this SAM3 confidence are left off the snapshot unless the pick tried them.
	snapshotMinConfidence = 0.75
)

// snapshotUploader stores a JPEG and returns its binary data ID.
type snapshotUploader func(ctx context.Context, jpeg []byte, tags []string) (string, error)

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

// recordPick uploads the snapshot for one pick_snack run. It never fails the
// pick: problems are logged and reported in result as snapshot_error.
func (s *snackRestockerRestocker) recordPick(
	ctx context.Context, scan *sam3Scan, chosen *snackTarget, status string, result map[string]interface{},
) {
	if scan == nil || scan.snapshotID == "" {
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
		return
	}

	upload := s.uploadSnapshot
	if upload == nil {
		upload = s.uploadSnapshotToData
	}
	id, err := upload(ctx, jpeg, []string{snapshotTag, snapshotTag + "_" + status})
	if err != nil {
		s.logger.Warnf("pick snapshot: upload: %v", err)
		set("snapshot_error", "upload: "+err.Error())
		return
	}
	s.logger.Infof("pick snapshot uploaded: binary data id %s (tags %s, %s_%s)", id, snapshotTag, snapshotTag, status)
	set("snapshot_binary_data_id", id)
}

// uploadSnapshotToData uploads to Viam data as this service's pick_snack
// capture, using the API key viam-server gives the module.
func (s *snackRestockerRestocker) uploadSnapshotToData(ctx context.Context, jpeg []byte, tags []string) (string, error) {
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
		ctx, jpeg, partID, "rdk:service:generic", s.name.ShortName(), snapshotMethod, ".jpg",
		&app.BinaryDataCaptureUploadOptions{Tags: tags, DataRequestTimes: &[2]time.Time{now, now}},
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
