OUT = GkMa.exe
BUILD_PATH = ""

.PHONY:

build: $(OUT)

$(OUT): $(OBJ)
	@mcs -unsafe -out:$(BUILD_PATH)/$(OUT) -pkg:dotnet -platform:anycpu -recurse:'*.cs'

clean:
	@rm -f $(OUT)

run: $(OUT)
	@mono $(OUT)
