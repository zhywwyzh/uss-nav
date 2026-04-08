#把当前所有文件的时间修改为当前系统时间
find . \( -path ./build -o -path ./devel -o -path ./.git \) -prune -o -type f -exec touch {} +
