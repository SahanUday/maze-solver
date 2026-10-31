# Simulator

[mms](https://github.com/mackorone/mms) is a Micromouse simulator. It loads a maze, starts a
maze-solving program as a child process, and moves a mouse through the maze as that program tells
it to. Here it is for testing maze logic and algorithms on a PC, not the whole robot: it moves one
tile at a time and has no ultrasonic, encoder, gyro or floor-colour model.

- Upstream: https://github.com/mackorone/mms (MIT). Protocol reference:
  [Mouse API](https://github.com/mackorone/mms#mouse-api).
- We use the prebuilt release **v1.2.0**, unmodified. Its source is not in this repo.
- What we plan to build on top of it: [TESTING-PLAN.md](TESTING-PLAN.md). Nothing there exists yet.

## Getting it

The binary is not in git (54 MB; the `check-added-large-files` hook stops anything over 1 MB, and
`*.AppImage` is git-ignored). It is attached to this repo's release
[sim-mms-v1.2.0](https://github.com/SahanUday/maze-solver/releases/tag/sim-mms-v1.2.0), an
unmodified copy of upstream's. From the repo root:

```
gh release download sim-mms-v1.2.0 -p mms-x86_64.AppImage -D sim/
sha256sum sim/mms-x86_64.AppImage
```

The hash must be:

```
d6278fb637a92ccc8e92cb807f12010603b6beb4564eb026630d9b4694a138a6
```

If the release is ever gone, the same file is inside `linux.zip` of upstream's
[v1.2.0](https://github.com/mackorone/mms/releases/tag/v1.2.0) (macOS and Windows builds are there
too).

## Running it

```
chmod +x sim/mms-x86_64.AppImage
sim/mms-x86_64.AppImage
```

On WSL2 the window needs WSLg, and the AppImage needs `libfuse2`; without FUSE the standard
AppImage option `--appimage-extract-and-run` is the usual workaround (not tried here). At launch on
our WSL2 machine it printed `Could not find the Qt platform plugin "wayland"` and kept running.

## Loading a maze

The **Maze** drop-down in the *Config* box lists the built-in mazes (`blank`, `example1`..`5`) and
every file you have loaded before. To add one, click the folder icon beside it and pick a file. The
dialog has no file-type filter and the extension does not matter: the content is read as either
format below. A file that fails shows an "Invalid Maze File" warning. A maze must be:

- non-empty and rectangular (any size, not only 16x16),
- fully enclosed by outer walls,
- consistent: a wall between two cells is marked on both.

**Map format** (draw it; the last line is the bottom row, `y = 0`):

```
+---+---+---+
|       |   |
+   +   +   +
|   |       |
+---+---+---+
```

A cell is 4 columns by 2 lines, borders shared. Any non-space character counts as a wall, but only
the wall positions are read, so text in the middle of a cell (a label such as `G`) is ignored. The
community collection in [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles)
uses this format with `o` for posts.

**Num format**, one line per cell, `X Y N E S W` with `1` for a wall (`(0,0)` is bottom left):

```
0 0 1 0 1 1
1 0 1 1 1 0
```

With no program running, the maze shows each cell's distance from the centre. The mouse always
starts on cell `(0,0)` facing North, and the goal is the centre cell(s), worked out from the maze
size. Neither can be set in the file.

## Using it

1. **Add a program.** Click the `+` beside **Mouse**. Fill in *Name*, *Directory* (the working
   directory), *Build Command* and *Run Command*. Commands are split like a shell line but are not
   run by a shell (no pipes, `&&` or `$VAR`); wrap paths with spaces in quotes.
2. **Build** runs the build command in that directory (output in *Build Output*). **Run** starts the
   run command. The pencil icon edits the entry. Settings are kept between sessions.
3. **Watch.** The *Controls* box has **Pause**, **Reset** and the speed slider (turtle to rabbit).
   While a program runs, the maze shows what the program has declared with `setWall`, `setColor`
   and `setText`; walls it has not declared show dark red. **Cancel** (the Run button while
   running) stops it and brings back the real maze.
4. **Reset** simulates a crash: the program should poll `wasReset`, clear its state and call
   `ackReset` to put the mouse back on the start.
5. Tabs: *Run Output* shows the program's **stderr** (log there), *Simulator Logs* shows commands mms
   did not understand, *Stats* shows distance, turns and a score.

### What a program does

mms and the program talk over stdin/stdout, one line per command, so a program must keep **stdout
for commands only**. A few of them:

```
mazeWidth          -> 16            size of the loaded maze
wallFront          -> true/false    also wallLeft, wallRight, wallBack
moveForward [N]    -> ack | crash   N tiles; crash if a wall is in the way
turnLeft / turnRight -> ack         90 degrees (45 degree turns exist too)
setWall 0 1 n      (no reply)       draw a wall the program believes in; n/e/s/w
setColor 0 1 g     (no reply)       colour a cell; setText 0 1 abc writes up to 10 characters
getStat total-turns -> 12           stats are read back the same way
```

The full list, the colour letters and the scoring rules are in the upstream README. Upstream also
has small client templates for several languages (C++, Python, ...); the Python one is a
left-wall follower and is a handy first check that the setup works.

### Limits that matter for us

- One mouse, one rectangular maze: no bridge, no second section. Test Section A and Section B as
  separate maze files.
- Walls are read perfectly. There is no sensor noise, no floor colour and no timing; "speed" is
  animation only.
- The stats and the score assume micromouse rules: a run starts when the mouse leaves `(0,0)` and
  ends in the centre. Our goal is elsewhere, so measure our own numbers in the program.
- It is a GUI program: it cannot run in CI, and it is slow for many-run statistics.
