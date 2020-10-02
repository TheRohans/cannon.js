.PHONY: all test clean

build:
	npm run build

publish:
	./publish.sh

clean:
	rm -rf lib
	rm -rf coverage

test:
	npm run test

release: clean test build publish
