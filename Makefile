
GO_BUILD_ENV :=
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/pointcloud-calibration

ifeq ($(VIAM_TARGET_OS), windows)
	GO_BUILD_ENV += GOOS=windows GOARCH=amd64
	GO_BUILD_FLAGS := -tags no_cgo
	MODULE_BINARY = bin/pointcloud-calibration.exe
endif

all: test module.tar.gz

clean:
	rm -f $(MODULE_BINARY)
	rm -f module.tar.gz

$(MODULE_BINARY): Makefile go.mod *.go cmd/module/*.go calibration/*.go pcutils/*.go
	$(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) cmd/module/main.go

lint:
	gofmt -s -w .

update:
	go get go.viam.com/rdk@latest
	go mod tidy

test:
	go test ./...

module.tar.gz: meta.json $(MODULE_BINARY)
ifeq ($(VIAM_TARGET_OS), windows)
	jq '.entrypoint = "./bin/pointcloud-calibration.exe"' meta.json > temp.json && mv temp.json meta.json
else
	strip $(MODULE_BINARY)
endif
	tar czf $@ meta.json $(MODULE_BINARY)
ifeq ($(VIAM_TARGET_OS), windows)
	git checkout meta.json
endif

module: module.tar.gz

setup:
	go mod tidy
	brew install nlopt-static || sudo apt install -y libnlopt-dev

license-check:
	license_finder
