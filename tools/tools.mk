.PHONY: build_tools
build_tools:
	gcc -o tools/main.o tools/main.c

.PHONY: run_tools
run_tools:
	tools/main.o tools/joint_input.json tools/joint_output.h

.PHONY: all_tools
all_tools: build_tools run_tools
