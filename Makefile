.DEFAULT_GOAL := help
.PHONY: help image start stop shell run logs pydeps

LAB  := scripts/lib.sh
ARGS ?=

# `make run <шлях>`: усе після run - шлях до місії, а не окрема ціль
ifeq (run,$(firstword $(MAKECMDGOALS)))
  PART := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
  $(eval $(PART):;@:)
endif

define HELP_HEADER
Використання: make <ціль> [ЗМІННА=значення]
               make run <шлях до місії>

Цілі:
endef

define HELP_FOOTER

Змінні:
  ARGS         аргументи ros2 launch для start або docker compose build для image
  LAUNCH_FILE  launch-файл для start (типово iris_maze.launch.py)
  QGC          1: start також запускає QGroundControl, stop закриває його
               і переносить .tlog у logs/lab_7/Telemetry

Приклади:
  make start ARGS=rviz:=false
  make start ARGS=rviz:=false QGC=1
  make stop QGC=1
  make run lab_2/plain_start
  make run lab_2/go_to
endef
export HELP_HEADER HELP_FOOTER

help: ## показати цю довідку
	@echo "$$HELP_HEADER"
	@grep -hE '^[a-zA-Z][a-zA-Z0-9_-]*:.*## ' $(MAKEFILE_LIST) \
	  | sort \
	  | awk 'BEGIN{FS = ":.*## "}{printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}'
	@echo "$$HELP_FOOTER"

image: ## зібрати Docker-образ (перший раз ~15–20 хв)
	@$(LAB) image $(ARGS)

start: ## запустити Gazebo + SITL + micro-ROS agent і дочекатися готовності до arm
	@$(LAB) start $(ARGS)

stop: ## зупинити симуляцію і видалити контейнер
	@$(LAB) stop

shell: ## термінал у контейнері із середовищем ROS 2
	@$(LAB) shell

run: ## запустити місію за шляхом відносно fpv_labs/src, журнал у logs/
	@$(LAB) run $(PART)

logs: ## стежити за останнім журналом симуляції
	@$(LAB) logs

pydeps: ## скопіювати Python-пакети ROS 2 з образу в .pydeps/ для редактора
	@$(LAB) pydeps
