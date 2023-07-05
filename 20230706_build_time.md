# build time memo

Autowareをフルビルドするときにかかる時間のメモ
`MAKEFLAGS="-j"`のオプションを立てると良いとか言われているけど、手元の計測では全然変わらなかった

## Spec

```bash
$ cat /proc/cpuinfo
model name : Intel(R) Xeon(R) W-2235 CPU @ 3.80GHz
```

12 core

```bash
$ cat /proc/meminfo
MemTotal:       65547816 kB
```

## Time

### colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

Summary: 286 packages finished [47min 28s]

### MAKEFLAGS="-j1" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

Summary: 286 packages finished [46min 44s]

### MAKEFLAGS="-j" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

Summary: 286 packages finished [46min 37s]

### MAKEFLAGS="-j4" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

Summary: 286 packages finished [46min 5s]
