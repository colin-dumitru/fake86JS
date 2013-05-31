emmake make
emcc bin/fake86.o -o bin/fake86.html --preload-file data -O2 -s ASM_JS=1 -s EXPORTED_FUNCTIONS="['_runBurst']" --js-library src/js/common.js