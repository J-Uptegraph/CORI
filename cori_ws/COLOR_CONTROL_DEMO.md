# CORI Color Control - Just Type the Color!

The most intuitive way to control CORI - just type a color name and watch CORI look right at it on the table!

## Quick Start

1. Run `./build.bash` and select option 5
2. Wait for Gazebo to show the table with colored objects
3. Type any color name!

## Example Session

```
╭─────────────────────────────────────────────────╮
│  🤖 CORI INTERACTIVE CONTROL                    │
├─────────────────────────────────────────────────┤
│  Type commands in plain English!                │
│  Try: 'red', 'blue', 'wave', 'sit'             │
│  🎨 Type any color name to look at it!          │
│  Type 'help' for all commands                  │
╰─────────────────────────────────────────────────╯

cori> red
👀 Looking at red (-0.62 radians)

cori> yellow
👀 Looking at yellow (-0.23 radians)

cori> green
👀 Looking at green (0.00 radians)

cori> blue
👀 Looking at blue (0.23 radians)

cori> purple
👀 Looking at purple (0.44 radians)

cori> black
👀 Looking at black (0.76 radians)
```

## Available Colors on the Table

From CORI's left to right:
- **Red** (far left)
- **Orange**
- **Yellow**
- **Green** (center)
- **Blue**
- **Purple**
- **Grey/Gray** (both spellings work!)
- **Black** (far right)

## How It Works

1. **You type a color name** - Just the color, nothing else needed
2. **CORI calculates the angle** - Uses math to figure out where that color is
3. **Head turns instantly** - CORI looks directly at that colored object
4. **Visual feedback** - You see both the angle and CORI turning in Gazebo

## The Math Behind It

CORI uses `atan2` to calculate the exact angle needed:
- CORI's head is at position (0, 0)
- Table is at X=1.5m with colors spread along the Y axis
- Each color has a fixed position on the table
- The angle is calculated in real-time when you type the color

## Fun Sequences to Try

### Rainbow Scan
```
cori> red
cori> orange
cori> yellow
cori> green
cori> blue
cori> purple
```

### Back and Forth
```
cori> red
cori> black
cori> red
cori> black
cori> green
```

### Color Hunt Game
```
cori> Find the blue one!
blue
cori> Now red!
red
cori> Yellow!
yellow
```

## Combine with Other Commands

You can mix color commands with regular commands:

```
cori> red
👀 Looking at red (-0.62 radians)

cori> wave
👋 Waving right arm

cori> blue
👀 Looking at blue (0.23 radians)

cori> sit
🪑 Sitting down

cori> green
👀 Looking at green (0.00 radians)

cori> stand
🧍 Standing up

cori> purple
👀 Looking at purple (0.44 radians)
```

## Why This Is Awesome

✅ **Just type the color** - No "look at" needed
✅ **Automatic calculation** - CORI does the math
✅ **Precise targeting** - Looks exactly at each object
✅ **Visual confirmation** - Watch the head turn in Gazebo
✅ **Natural language** - Say it like you think it
✅ **Instant response** - No delay, immediate movement

## Technical Details

- **Coordinate System**: CORI at origin, table at (1.5, 0) rotated 90°
- **Angle Range**: Head joint limited to -1.8 to 1.8 radians (~±103°)
- **Calculation**: Uses `math.atan2(color_y, table_x)` for precise angles
- **Colors Supported**: 8 colors (9 including grey/gray spelling)

## Pro Tips

1. **Spell it right**: Color names must be lowercase and spelled correctly
2. **One word only**: Just type the color, no extra words needed
3. **Watch Gazebo**: You'll see CORI's head turn to face the color
4. **Try grey or gray**: Both spellings work!
5. **Mix it up**: Use colors with other commands for complex sequences

---

**This is the future of robot control - natural, intuitive, and instant!** 🎨🤖
