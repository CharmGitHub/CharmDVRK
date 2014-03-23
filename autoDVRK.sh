mkdir Core-Build
cd Core-Build
cmake ../Core
make
cd ..

mkdir Applications-Build
cd Applications-Build
cmake ../Applications
make
cd ..
