package snackrestocker

import (
	"context"
	"errors"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
)

// PickStatus is a sensor whose readings are a restocker's live pick status,
// so the app's CONTROL tab, data capture and triggers can follow a pick as it
// runs. It talks to the restocker only through get_pick_status.
var PickStatus = resource.NewModel("viam", "snack-restocker", "pick-status")

func init() {
	resource.RegisterComponent(sensor.API, PickStatus,
		resource.Registration[sensor.Sensor, *PickStatusConfig]{
			Constructor: newPickStatus,
		},
	)
}

// PickStatusConfig names the restocker service to report on.
type PickStatusConfig struct {
	Restocker string `json:"restocker"`
}

// Validate requires the restocker, which is also the sensor's dependency.
func (cfg *PickStatusConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Restocker == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "restocker")
	}
	return []string{cfg.Restocker}, nil, nil
}

type pickStatusSensor struct {
	resource.Named
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	restocker resource.Resource
}

func newPickStatus(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (sensor.Sensor, error) {
	conf, err := resource.NativeConfig[*PickStatusConfig](rawConf)
	if err != nil {
		return nil, err
	}
	restocker, err := generic.FromProvider(deps, conf.Restocker)
	if err != nil {
		return nil, err
	}
	return &pickStatusSensor{Named: rawConf.ResourceName().AsNamed(), restocker: restocker}, nil
}

// Readings is the restocker's get_pick_status without the event history, so
// each captured reading stays small; the history is in get_pick_status.
func (p *pickStatusSensor) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	status, err := p.restocker.DoCommand(ctx, map[string]interface{}{"get_pick_status": true})
	if err != nil {
		return nil, err
	}
	if status == nil {
		return nil, errors.New("restocker returned no pick status")
	}
	readings := make(map[string]interface{}, len(status))
	for k, v := range status {
		if k != "events" {
			readings[k] = v
		}
	}
	return readings, nil
}

func (p *pickStatusSensor) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return p.restocker.DoCommand(ctx, map[string]interface{}{"get_pick_status": true})
}
