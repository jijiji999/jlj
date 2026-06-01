from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "dm_motor_mock_screenshot.png"
WIDTH = 1600
HEIGHT = 2200


def font(size: int, bold: bool = False):
  candidates = []
  if bold:
    candidates.extend(
      [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
      ]
    )
  else:
    candidates.extend(
      [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
      ]
    )

  for candidate in candidates:
    path = Path(candidate)
    if path.exists():
      return ImageFont.truetype(str(path), size=size)
  return ImageFont.load_default()


BG = "#f3efe5"
CARD = (255, 251, 245, 236)
BORDER = "#e0d2c1"
TEXT = "#1f1a15"
MUTED = "#64584a"
ACCENT = "#1e5b83"
ACCENT2 = "#bd6d24"
SUCCESS = "#1b7f56"
WARNING = "#bc2f2f"
DISABLED = "#715544"

F_EYEBROW = font(18, bold=True)
F_H1 = font(54, bold=True)
F_BODY = font(22)
F_META = font(18)
F_CARD_TITLE = font(28, bold=True)
F_CARD_META = font(17)
F_STATE_LABEL = font(14)
F_STATE_VALUE = font(22, bold=True)
F_BUTTON = font(16, bold=True)
F_GROUP_TITLE = font(34, bold=True)
F_GROUP_DESC = font(20)
F_STAT = font(30, bold=True)


def rounded(draw, xy, radius, fill, outline=None, width=1):
  draw.rounded_rectangle(xy, radius=radius, fill=fill, outline=outline, width=width)


def text(draw, pos, value, fill, fnt):
  draw.text(pos, value, fill=fill, font=fnt)


def pill(draw, xy, label, bg, fg):
  rounded(draw, xy, 20, bg)
  bbox = draw.textbbox((0, 0), label, font=F_META)
  tw = bbox[2] - bbox[0]
  th = bbox[3] - bbox[1]
  x1, y1, x2, y2 = xy
  tx = x1 + (x2 - x1 - tw) / 2
  ty = y1 + (y2 - y1 - th) / 2 - 1
  draw.text((tx, ty), label, fill=fg, font=F_META)


def button(draw, xy, label, bg):
  rounded(draw, xy, 20, bg)
  bbox = draw.textbbox((0, 0), label, font=F_BUTTON)
  tw = bbox[2] - bbox[0]
  th = bbox[3] - bbox[1]
  x1, y1, x2, y2 = xy
  draw.text((x1 + (x2 - x1 - tw) / 2, y1 + (y2 - y1 - th) / 2 - 1), label, fill="white", font=F_BUTTON)


def state_box(draw, x, y, w, h, label, value):
  rounded(draw, (x, y, x + w, y + h), 16, "#ffffff")
  text(draw, (x + 14, y + 10), label, MUTED, F_STATE_LABEL)
  text(draw, (x + 14, y + 32), value, TEXT, F_STATE_VALUE)


def draw_range(draw, x, y, w, label, value, fill_ratio, locked=False, unlocked=False):
  text(draw, (x, y), label, MUTED, F_META)
  bbox = draw.textbbox((0, 0), value, font=F_META)
  vw = bbox[2] - bbox[0]
  text(draw, (x + w - vw - 28, y), value, TEXT, F_META)
  if locked or unlocked:
    lock_bg = "#2ca35e" if unlocked else "#b7b7b7"
    rounded(draw, (x + w - 20, y + 3, x + w - 2, y + 21), 4, lock_bg, outline=lock_bg)
  line_y = y + 34
  draw.rounded_rectangle((x, line_y, x + w, line_y + 8), radius=4, fill="#ddd1c4")
  fill_w = int(w * max(0.0, min(1.0, fill_ratio)))
  draw.rounded_rectangle((x, line_y, x + fill_w, line_y + 8), radius=4, fill=ACCENT)
  knob_x = x + fill_w
  draw.ellipse((knob_x - 8, line_y - 5, knob_x + 8, line_y + 11), fill="#fffdf8", outline="#cab9a6")
  if locked:
    overlay = Image.new("RGBA", (w, 8), (255, 255, 255, 90))
    return overlay, (x, line_y)
  return None


def create_background():
  image = Image.new("RGBA", (WIDTH, HEIGHT), BG)
  # Warm and cool background blooms.
  accent = Image.new("RGBA", (WIDTH, HEIGHT), (0, 0, 0, 0))
  ad = ImageDraw.Draw(accent)
  ad.ellipse((-120, -120, 520, 480), fill=(205, 134, 63, 54))
  ad.ellipse((1080, -140, 1720, 360), fill=(18, 90, 145, 42))
  ad.ellipse((1240, 1520, 1820, 2200), fill=(205, 134, 63, 34))
  image = Image.alpha_composite(image, accent)
  return image, ImageDraw.Draw(image)


def section_header(draw, x, y, title, desc, count):
  text(draw, (x, y), title, TEXT, F_GROUP_TITLE)
  text(draw, (x, y + 42), desc, MUTED, F_GROUP_DESC)
  bbox = draw.textbbox((0, 0), count, font=F_BODY)
  tw = bbox[2] - bbox[0]
  text(draw, (WIDTH - 90 - tw, y + 16), count, TEXT, F_BODY)


def big_motor_card(image, x, y, w, h, data):
  draw = ImageDraw.Draw(image)
  rounded(draw, (x, y, x + w, y + h), 24, CARD, outline=BORDER)

  text(draw, (x + 20, y + 18), data["bus"], MUTED, F_CARD_META)
  text(draw, (x + 20, y + 42), data["name"], TEXT, F_CARD_TITLE)
  text(draw, (x + 20, y + 78), data["meta"], MUTED, F_CARD_META)

  if data["status"] == "enabled":
    pill(draw, (x + w - 120, y + 20, x + w - 20, y + 56), "enabled", (27, 127, 86, 36), SUCCESS)
  elif data["status"] == "disabled":
    pill(draw, (x + w - 126, y + 20, x + w - 20, y + 56), "disabled", (100, 88, 74, 30), MUTED)
  else:
    pill(draw, (x + w - 92, y + 20, x + w - 20, y + 56), "fault", (188, 47, 47, 28), WARNING)

  bx = x + 20
  by = y + 118
  bw = 108
  bh = 38
  gap = 10
  button(draw, (bx, by, bx + bw, by + bh), "Enable", ACCENT)
  button(draw, (bx + (bw + gap), by, bx + 2 * bw + gap, by + bh), "Disable", DISABLED)
  button(draw, (bx + 2 * (bw + gap), by, bx + 3 * bw + 2 * gap, by + bh), "Clear Error", ACCENT2)
  button(draw, (bx + 3 * (bw + gap), by, bx + 4 * bw + 3 * gap, by + bh), "Save Zero", ACCENT2)

  sx = x + 20
  sy = y + 178
  sw = (w - 20 * 2 - 12 * 2) / 3
  sh = 74
  labels = data["state"]
  for idx, (label, value) in enumerate(labels[:6]):
    row = idx // 3
    col = idx % 3
    state_box(draw, int(sx + col * (sw + 12)), int(sy + row * (sh + 12)), int(sw), sh, label, value)

  ry = y + 352
  overlays = []
  overlays.append(draw_range(draw, x + 20, ry, w - 40, "Position", data["ranges"][0][0], data["ranges"][0][1]))
  overlays.append(draw_range(draw, x + 20, ry + 64, w - 40, "Velocity", data["ranges"][1][0], data["ranges"][1][1]))
  overlays.append(draw_range(draw, x + 20, ry + 128, w - 40, "Kp", data["ranges"][2][0], data["ranges"][2][1], locked=data["ranges"][2][2], unlocked=data["ranges"][2][3]))
  overlays.append(draw_range(draw, x + 20, ry + 192, w - 40, "Kd", data["ranges"][3][0], data["ranges"][3][1], locked=data["ranges"][3][2], unlocked=data["ranges"][3][3]))
  overlays.append(draw_range(draw, x + 20, ry + 256, w - 40, "Torque", data["ranges"][4][0], data["ranges"][4][1]))

  for overlay in overlays:
    if overlay:
      image.alpha_composite(overlay[0], dest=overlay[1])

  my = y + h - 70
  button(draw, (x + 20, my, x + 136, my + 38), "Single Send", SUCCESS)
  toggle_bg = WARNING if data["continuous"] else "#8a5b20"
  button(draw, (x + 146, my, x + 282, my + 38), "Continuous On" if data["continuous"] else "Continuous Off", toggle_bg)
  button(draw, (x + 292, my, x + 398, my + 38), "Reset Zero", ACCENT2)
  msg_color = SUCCESS if data["message_kind"] == "success" else (WARNING if data["message_kind"] == "error" else MUTED)
  text(draw, (x + 20, my + 46), data["message"], msg_color, F_META)


def compact_card(draw, x, y, w, h, data):
  rounded(draw, (x, y, x + w, y + h), 22, CARD, outline=BORDER)
  text(draw, (x + 16, y + 14), data["bus"], MUTED, F_CARD_META)
  text(draw, (x + 16, y + 36), data["name"], TEXT, font(22, bold=True))
  text(draw, (x + 16, y + 66), data["meta"], MUTED, F_CARD_META)
  if data["status"] == "enabled":
    pill(draw, (x + w - 98, y + 16, x + w - 16, y + 48), "enabled", (27, 127, 86, 36), SUCCESS)
  elif data["status"] == "disabled":
    pill(draw, (x + w - 104, y + 16, x + w - 16, y + 48), "disabled", (100, 88, 74, 30), MUTED)
  else:
    pill(draw, (x + w - 80, y + 16, x + w - 16, y + 48), "fault", (188, 47, 47, 28), WARNING)

  grid_x = x + 16
  grid_y = y + 102
  grid_w = (w - 16 * 2 - 12) / 2
  grid_h = 64
  for idx, (label, value) in enumerate(data["state"]):
    row = idx // 2
    col = idx % 2
    state_box(draw, int(grid_x + col * (grid_w + 12)), int(grid_y + row * (grid_h + 10)), int(grid_w), grid_h, label, value)


def main():
  image, draw = create_background()

  text(draw, (52, 46), "DM MOTOR VISUAL TEST BENCH", ACCENT, F_EYEBROW)
  text(draw, (52, 86), "Temporary Mock Panel for Screenshot Capture", TEXT, F_H1)
  hero_lines = [
    "当前设备未连接，本页面仅用于截图展示。界面布局、分组方式和控制项尽量对齐真实",
    "dm_motor 可视化调试面板，适合临时做汇报、文档或演示素材。",
  ]
  for i, line in enumerate(hero_lines):
    text(draw, (52, 165 + i * 34), line, MUTED, F_BODY)

  meta_x = 980
  meta_y = 66
  meta_w = 176
  meta_h = 108
  meta_gap = 14
  stats = [("Backend", "Mock Connected"), ("Motors", "29"), ("Auto Refresh", "100 ms")]
  for i, (label, value) in enumerate(stats):
    x = meta_x + i * (meta_w + meta_gap)
    rounded(draw, (x, meta_y, x + meta_w, meta_y + meta_h), 20, CARD, outline=BORDER)
    text(draw, (x + 18, meta_y + 18), label, MUTED, F_META)
    text(draw, (x + 18, meta_y + 52), value, TEXT, F_STAT)

  rounded(draw, (52, 272, WIDTH - 52, 348), 18, CARD, outline=BORDER)
  button(draw, (74, 291, 210, 329), "Refresh State", (232, 241, 247))
  draw.text((105, 302), "Refresh State", fill=ACCENT, font=F_BUTTON)
  button(draw, (224, 291, 392, 329), "Stop All Continuous", (232, 241, 247))
  draw.text((246, 302), "Stop All Continuous", fill=ACCENT, font=F_BUTTON)
  text(draw, (1088, 301), "Continuous Send Rate", MUTED, F_META)
  rounded(draw, (1314, 289, 1398, 331), 22, "#fffdf8", outline=BORDER)
  text(draw, (1343, 299), "20", TEXT, F_META)
  text(draw, (1410, 301), "Hz", MUTED, F_META)

  current_y = 380
  # Torso section.
  rounded(draw, (52, current_y, WIDTH - 52, current_y + 690), 24, CARD, outline=BORDER)
  section_header(draw, 78, current_y + 24, "Torso", "Waist joints and upper trunk drives", "3 motors")

  torso_cards = [
    {
      "bus": "vcan0",
      "name": "waist_yaw",
      "meta": "10010 | CAN 0x01",
      "status": "enabled",
      "state": [
        ("Position", "0.184"),
        ("Velocity", "0.012"),
        ("Torque", "1.920"),
        ("MOS Temp", "34.2"),
        ("Rotor Temp", "37.8"),
        ("Bus", "vcan0"),
      ],
      "ranges": [
        ("0.180", 0.53),
        ("0.020", 0.50),
        ("27.000", 0.05, True, False),
        ("3.700", 0.74, True, False),
        ("2.000", 0.51),
      ],
      "continuous": True,
      "message": "Continuous send running at 20 Hz",
      "message_kind": "success",
    },
    {
      "bus": "vcan1",
      "name": "waist_parallel_1",
      "meta": "8009P | CAN 0x01",
      "status": "disabled",
      "state": [
        ("Position", "-0.032"),
        ("Velocity", "0.000"),
        ("Torque", "0.000"),
        ("MOS Temp", "31.4"),
        ("Rotor Temp", "33.0"),
        ("Bus", "vcan1"),
      ],
      "ranges": [
        ("0.000", 0.50),
        ("0.000", 0.50),
        ("27.000", 0.05, True, False),
        ("3.700", 0.74, True, False),
        ("0.000", 0.50),
      ],
      "continuous": False,
      "message": "Ready for command",
      "message_kind": "neutral",
    },
    {
      "bus": "vcan1",
      "name": "waist_parallel_2",
      "meta": "8009P | CAN 0x02",
      "status": "fault",
      "state": [
        ("Position", "-0.118"),
        ("Velocity", "-0.425"),
        ("Torque", "4.221"),
        ("MOS Temp", "48.6"),
        ("Rotor Temp", "55.1"),
        ("Bus", "vcan1"),
      ],
      "ranges": [
        ("-0.120", 0.49),
        ("-0.400", 0.49),
        ("27.000", 0.20, False, True),
        ("2.100", 0.42, False, True),
        ("4.500", 0.58),
      ],
      "continuous": False,
      "message": "Driver reported over-temperature warning",
      "message_kind": "error",
    },
  ]

  card_w = 468
  card_h = 570
  gap = 18
  start_x = 78
  start_y = current_y + 98
  for idx, card in enumerate(torso_cards):
    big_motor_card(image, start_x + idx * (card_w + gap), start_y, card_w, card_h, card)

  current_y += 720
  rounded(draw, (52, current_y, WIDTH - 52, current_y + 500), 24, CARD, outline=BORDER)
  section_header(draw, 78, current_y + 24, "Left Arm", "Left shoulder, elbow and wrist chain", "7 motors")

  left_cards = [
    ("vcan0", "left_shoulder_pitch", "8009P | CAN 0x02", "enabled", [("Position", "-1.204"), ("Velocity", "0.115"), ("Torque", "3.102"), ("Bus", "vcan0")]),
    ("vcan0", "left_shoulder_roll", "8009P | CAN 0x03", "enabled", [("Position", "-0.814"), ("Velocity", "0.083"), ("Torque", "2.884"), ("Bus", "vcan0")]),
    ("vcan0", "left_shoulder_yaw", "4340P | CAN 0x04", "disabled", [("Position", "0.042"), ("Velocity", "0.000"), ("Torque", "0.000"), ("Bus", "vcan0")]),
    ("vcan0", "left_elbow", "4340 | CAN 0x05", "enabled", [("Position", "-0.336"), ("Velocity", "0.291"), ("Torque", "5.228"), ("Bus", "vcan0")]),
    ("vcan0", "left_wrist_roll", "4310 | CAN 0x06", "enabled", [("Position", "0.147"), ("Velocity", "-0.214"), ("Torque", "0.882"), ("Bus", "vcan0")]),
    ("vcan0", "left_wrist_pitch", "4310 | CAN 0x07", "enabled", [("Position", "-0.078"), ("Velocity", "0.119"), ("Torque", "0.462"), ("Bus", "vcan0")]),
    ("vcan0", "left_wrist_yaw", "4310 | CAN 0x08", "disabled", [("Position", "0.012"), ("Velocity", "0.000"), ("Torque", "0.000"), ("Bus", "vcan0")]),
  ]

  compact_w = 330
  compact_h = 176
  compact_gap_x = 18
  compact_gap_y = 18
  base_x = 78
  base_y = current_y + 98
  for idx, item in enumerate(left_cards):
    row = idx // 4
    col = idx % 4
    compact_card(
      draw,
      base_x + col * (compact_w + compact_gap_x),
      base_y + row * (compact_h + compact_gap_y),
      compact_w,
      compact_h,
      {
        "bus": item[0],
        "name": item[1],
        "meta": item[2],
        "status": item[3],
        "state": item[4],
      },
    )

  current_y += 530
  rounded(draw, (52, current_y, WIDTH - 52, current_y + 460), 24, CARD, outline=BORDER)
  section_header(draw, 78, current_y + 24, "Right Leg", "Right hip, knee and ankle chain", "6 motors")

  right_cards = [
    ("vcan3", "right_hip_pitch", "10010 | CAN 0x01", "enabled", [("Position", "0.622"), ("Velocity", "-0.091"), ("Torque", "12.408"), ("Bus", "vcan3")]),
    ("vcan3", "right_hip_roll", "10010 | CAN 0x02", "enabled", [("Position", "-0.214"), ("Velocity", "0.065"), ("Torque", "9.772"), ("Bus", "vcan3")]),
    ("vcan3", "right_hip_yaw", "10010 | CAN 0x03", "enabled", [("Position", "0.118"), ("Velocity", "0.043"), ("Torque", "7.605"), ("Bus", "vcan3")]),
    ("vcan3", "right_knee", "10010 | CAN 0x04", "enabled", [("Position", "-0.962"), ("Velocity", "0.152"), ("Torque", "16.030"), ("Bus", "vcan3")]),
    ("vcan3", "right_ankle_parallel_1", "4340 | CAN 0x05", "disabled", [("Position", "0.034"), ("Velocity", "0.000"), ("Torque", "0.000"), ("Bus", "vcan3")]),
    ("vcan3", "right_ankle_parallel_2", "4340 | CAN 0x06", "fault", [("Position", "-0.147"), ("Velocity", "-0.806"), ("Torque", "6.812"), ("Bus", "vcan3")]),
  ]

  base_y = current_y + 98
  for idx, item in enumerate(right_cards):
    row = idx // 4
    col = idx % 4
    compact_card(
      draw,
      base_x + col * (compact_w + compact_gap_x),
      base_y + row * (compact_h + compact_gap_y),
      compact_w,
      compact_h,
      {
        "bus": item[0],
        "name": item[1],
        "meta": item[2],
        "status": item[3],
        "state": item[4],
      },
    )

  image.convert("RGB").save(OUT, quality=95)
  print(OUT)


if __name__ == "__main__":
  main()
