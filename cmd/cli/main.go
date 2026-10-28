package main

import (
	"context"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
	"snackrestocker"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}

func realMain() error {
	ctx := context.Background()
	logger := logging.NewLogger("cli")

	deps := resource.Dependencies{}
	// can load these from a remote machine if you need

	cfg := snackrestocker.Config{}

	thing, err := snackrestocker.NewRestocker(ctx, deps, generic.Named("foo"), cfg, logger)
	if err != nil {
		return err
	}
	defer thing.Close(ctx)

	return nil
}
