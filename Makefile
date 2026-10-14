# Everyday commands; everything runs inside Docker, so the host needs only git, make and Docker.
# `make` without arguments prints the list. PKG=<package> limits build/test to one package.

SHELL := /bin/bash
.DEFAULT_GOAL := help

export HOST_UID := $(shell id -u)
export HOST_GID := $(shell id -g)

COMPOSE := docker compose
PKG ?=
DEV_RUN := $(COMPOSE) run --rm dev bash -c
DEV_ENV := source /etc/capstone_dev.sh
BUILD_ARGS := --symlink-install $(if $(PKG),--packages-up-to $(PKG)) \
	--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
	-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
TEST_ARGS := $(if $(PKG),--packages-select $(PKG))
TEST_RESULT_ARGS := $(if $(PKG),--test-result-base build/$(PKG))

.PHONY: help setup doctor images build test shell smoke sim sim-gui rqt-graph rqt rviz g2-dataset g2-labels g2-dataset-splits g2-train g2-eval g2-model down clean dev-image dirs

help: ## Показать эту справку
	@echo "Команды проекта (PKG=имя_пакета ограничивает build/test одним пакетом):"
	@grep -E '^[a-z0-9-]+:.*## ' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*## "} {printf "  make %-17s %s\n", $$1, $$2}'

setup: ## Первый запуск: проверка компьютера, образы, сборка, тесты и проверка сцены Webots
	@scripts/setup_host.sh --check
	@$(MAKE) --no-print-directory images
	@$(MAKE) --no-print-directory build
	@$(MAKE) --no-print-directory test
	@$(MAKE) --no-print-directory smoke
	@echo
	@echo "ГОТОВО: окружение работает. Список команд: make"

doctor: ## Проверить компьютер и подсказать, что исправить
	@scripts/setup_host.sh --check

images: dirs ## Пересобрать все Docker-образы (обычно не нужно: make build делает это сам)
	$(COMPOSE) --profile dev build
	@scripts/dev_image.sh --mark

dev-image: dirs
	@scripts/dev_image.sh

dirs:
	@mkdir -p .cache/home .cache/ccache

build: dev-image ## Собрать код (быстро, собирается только изменённое)
	$(DEV_RUN) '$(DEV_ENV) && colcon build $(BUILD_ARGS)'

test: build ## Собрать код и запустить тесты
	$(DEV_RUN) '$(DEV_ENV) && colcon test $(TEST_ARGS) --event-handlers console_direct+ \
		&& colcon test-result --verbose $(TEST_RESULT_ARGS)'
	$(if $(PKG),,$(DEV_RUN) 'python3 -m unittest discover -s /workspace/ml/perception')

shell: dev-image ## Открыть терминал в контейнере разработки (ros2, colcon)
	$(COMPOSE) run --rm dev bash

smoke: ## Проверка как в CI: сцена Webots загружается, /clock доходит до второго контейнера
	$(COMPOSE) build simulator autonomy
	@status=0; timeout 120s $(COMPOSE) up --abort-on-container-exit --exit-code-from autonomy || status=$$?; \
	$(COMPOSE) down --remove-orphans >/dev/null 2>&1; \
	if [ $$status -eq 0 ]; then echo "PASS: сцена Webots запустилась, /clock идёт"; \
	else echo "FAIL: проверка не прошла (код $$status), причина — в логе выше"; fi; exit $$status

sim: ## Запустить симулятор без окна (остановка: Ctrl+C)
	$(COMPOSE) build simulator
	@status=0; $(COMPOSE) run --rm -v "$(CURDIR)/simulation:/opt/capstone/simulation:ro" simulator || status=$$?; \
	[ $$status -eq 0 ] || [ $$status -eq 130 ] || exit $$status

sim-gui: ## Открыть сцену в окне Webots
	@scripts/gui.sh simulator

rqt-graph: dev-image ## Граф узлов и топиков ROS (rqt_graph)
	@scripts/gui.sh gui rqt_graph

rqt: dev-image ## Инструменты rqt: изображения камеры, дерево TF, топики, логи
	@scripts/gui.sh gui rqt

rviz: dev-image ## RViz с конфигурацией проекта (rviz/capstone.rviz)
	@scripts/gui.sh gui rviz2 -d /workspace/rviz/capstone.rviz

# G2, method B: training data (ml/perception/README.md).
SAMPLES ?= 200
SEED ?= 0
LAYOUT ?=
RUN ?= $(if $(LAYOUT),$(LAYOUT),warehouse_seed$(SEED))

g2-dataset: ## G2: записать кадры RGB-D в data/g2/RUN (SAMPLES=200; LAYOUT=train_0 или SEED=0 для warehouse.wbt)
	$(COMPOSE) build simulator
	@mkdir -p data/g2
	$(COMPOSE) run --rm -u "$(HOST_UID):$(HOST_GID)" -e HOME=/tmp -e RUN="$(RUN)" -e LAYOUT="$(LAYOUT)" \
		-e GIT_COMMIT="$$(git rev-parse HEAD)" -e GIT_DIRTY="$$(test -z "$$(git status --porcelain)" && echo false || echo true)" \
		-v "$(CURDIR)/ml/perception:/opt/g2:ro" -v "$(CURDIR)/data/g2:/data" simulator \
		bash /opt/g2/run_recording.sh --samples $(SAMPLES) --seed $(SEED)

g2-labels: dev-image ## G2: разметить записанные кадры data/g2/RUN по чистой глубине
	$(DEV_RUN) 'cd /workspace/ml/perception && python3 label_dataset.py /workspace/data/g2/$(RUN)'

TRAIN_ARGS ?=
EVAL_ARGS ?=

g2-train: dirs ## G2: обучить CNN метода B на data/g2 (видеокарта, если есть nvidia-container-toolkit; TRAIN_ARGS=...)
	@files="-f compose.yml"; \
	if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then files="$$files -f compose.gpu.yml"; \
	else echo ">>> nvidia-container-toolkit не найден: обучение на CPU"; fi; \
	$(COMPOSE) $$files --profile train build train && \
	$(COMPOSE) $$files --profile train run --rm train python3 train.py $(TRAIN_ARGS)

g2-eval: dirs ## G2: проверить data/g2/models/latest на тестовых складах (EVAL_ARGS=...)
	$(COMPOSE) --profile train build train
	$(COMPOSE) --profile train run --rm train python3 evaluate.py $(EVAL_ARGS)

g2-model: dirs ## G2: положить data/g2/models/latest в пакет capstone_perception (модель и эталон теста C++)
	$(COMPOSE) --profile train build train
	$(COMPOSE) --profile train run --rm train python3 release_model.py

g2-dataset-splits: ## G2: записать и разметить все планировки из ml/perception/splits.json (SAMPLES на каждую)
	@for layout in $$(python3 ml/perception/generate_layout.py --list); do \
		$(MAKE) --no-print-directory g2-dataset LAYOUT=$$layout RUN=$$layout SAMPLES=$(SAMPLES) && \
		$(MAKE) --no-print-directory g2-labels RUN=$$layout || exit 1; \
	done

down: ## Остановить все контейнеры проекта
	$(COMPOSE) --profile dev --profile gui down --remove-orphans

clean: ## Удалить результаты сборки (ros2_ws/build, install, log)
	rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
