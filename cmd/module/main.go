package main

import (
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
	"snackrestocker"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: generic.API, Model: snackrestocker.Restocker},
		resource.APIModel{API: sensor.API, Model: snackrestocker.PickStatus},
	)
}
