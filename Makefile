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

.PHONY: help setup doctor images build test shell smoke sim sim-gui down clean dev-image dirs

help: ## Показать эту справку
	@echo "Команды проекта (PKG=имя_пакета ограничивает build/test одним пакетом):"
	@grep -E '^[a-z-]+:.*## ' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*## "} {printf "  make %-9s %s\n", $$1, $$2}'

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

test: dev-image ## Запустить тесты
	$(DEV_RUN) '$(DEV_ENV) && colcon test $(TEST_ARGS) --event-handlers console_direct+ \
		&& colcon test-result --verbose $(TEST_RESULT_ARGS)'

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
	$(COMPOSE) build simulator
	@scripts/sim_gui.sh

down: ## Остановить все контейнеры проекта
	$(COMPOSE) --profile dev down --remove-orphans

clean: ## Удалить результаты сборки (ros2_ws/build, install, log)
	rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
