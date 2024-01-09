# Target binary name
OUT=CPU

# Source files
SOURCES=PipelineCPU/template_lab2.v $(filter-out PipelineCPU/template_lab2.v, $(wildcard PipelineCPU/*.v))


# Rule for the default target
all: $(OUT)

# Compile rule
$(OUT): $(SOURCES)
	iverilog -o $(OUT) $(SOURCES)

# Clean rule to remove generated files
clean:
	rm -f $(OUT)

.PHONY: all clean
