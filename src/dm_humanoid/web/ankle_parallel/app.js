const backendStatus = document.getElementById("backend-status");
const panels = Array.from(document.querySelectorAll(".ankle-panel"));

const format = (value, digits = 3) => Number(value ?? 0).toFixed(digits);

const apiPost = async (path, payload = {}) => {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  return response.json();
};

const panelState = new Map();

const setMessage = (element, text, success) => {
  element.textContent = text;
  element.classList.remove("is-error", "is-success");
  element.classList.add(success ? "is-success" : "is-error");
};

const currentMode = (panel) => panel.querySelector(".mode-radio:checked")?.value ?? "joint";

const syncPanelLabels = (panel) => {
  panel.querySelector(".pitch-value").textContent = format(panel.querySelector(".pitch-slider").value);
  panel.querySelector(".roll-value").textContent = format(panel.querySelector(".roll-slider").value);
  panel.querySelector(".kp-value").textContent = format(panel.querySelector(".kp-slider").value);
  panel.querySelector(".kd-value").textContent = format(panel.querySelector(".kd-slider").value);
  panel.querySelector(".motor1-value").textContent = format(panel.querySelector(".motor1-slider").value);
  panel.querySelector(".motor2-value").textContent = format(panel.querySelector(".motor2-slider").value);
};

const applyModeState = (panel) => {
  const jointMode = currentMode(panel) === "joint";
  panel.querySelector(".pitch-slider").disabled = !jointMode;
  panel.querySelector(".roll-slider").disabled = !jointMode;
  panel.querySelector(".motor1-slider").disabled = jointMode;
  panel.querySelector(".motor2-slider").disabled = jointMode;
};

const buildSolvePayload = (panel) => {
  const side = panel.dataset.side;
  const mode = currentMode(panel);
  if (mode === "joint") {
    return {
      side,
      command_mode: "joint",
      pitch: Number(panel.querySelector(".pitch-slider").value),
      roll: Number(panel.querySelector(".roll-slider").value),
    };
  }

  return {
    side,
    command_mode: "motor",
    motor_1_position: Number(panel.querySelector(".motor1-slider").value),
    motor_2_position: Number(panel.querySelector(".motor2-slider").value),
  };
};

const buildSendPayload = (panel, solved) => ({
  side: panel.dataset.side,
  motor_1_position: solved.motors.motor_1_position,
  motor_2_position: solved.motors.motor_2_position,
  kp: Number(panel.querySelector(".kp-slider").value),
  kd: Number(panel.querySelector(".kd-slider").value),
});

const setPreview = (panel, result, success) => {
  const preview = panel.querySelector(".preview-message");
  preview.textContent = result;
  preview.classList.remove("is-error", "is-success");
  preview.classList.add(success ? "is-success" : "is-error");
};

const applySolveResultToInputs = (panel, result) => {
  panel.querySelector(".pitch-slider").value = result.logical.pitch;
  panel.querySelector(".roll-slider").value = result.logical.roll;
  panel.querySelector(".motor1-slider").value = result.motors.motor_1_position;
  panel.querySelector(".motor2-slider").value = result.motors.motor_2_position;
  syncPanelLabels(panel);
};

const updatePanelState = (panel, data) => {
  panel.querySelector(".state-pitch").textContent = format(data.logical.pitch);
  panel.querySelector(".state-roll").textContent = format(data.logical.roll);
  panel.querySelector(".state-m1-pos").textContent = format(data.motors.motor_1.position);
  panel.querySelector(".state-m2-pos").textContent = format(data.motors.motor_2.position);
  panel.querySelector(".state-m1-vel").textContent = format(data.motors.motor_1.velocity);
  panel.querySelector(".state-m2-vel").textContent = format(data.motors.motor_2.velocity);
  panel.querySelector(".state-m1-torque").textContent = format(data.motors.motor_1.torque);
  panel.querySelector(".state-m2-torque").textContent = format(data.motors.motor_2.torque);
  panel.querySelector(".state-message").textContent = data.message ?? "OK";
};

const refreshState = async () => {
  try {
    const response = await fetch("/api/state");
    const data = await response.json();
    backendStatus.textContent = "Connected";
    panels.forEach((panel) => {
      const sideData = data[panel.dataset.side];
      if (sideData) {
        updatePanelState(panel, sideData);
      }
    });
  } catch (error) {
    backendStatus.textContent = "Offline";
    panels.forEach((panel) => {
      panel.querySelector(".state-message").textContent = "State refresh failed";
    });
    console.error(error);
  }
};

const solvePanel = async (panel) => {
  const payload = buildSolvePayload(panel);
  const result = await apiPost("/api/solve", payload);
  const sideState = panelState.get(panel.dataset.side) ?? {};

  if (!result.success) {
    panelState.set(panel.dataset.side, { ...sideState, solvedResult: null });
    setPreview(panel, result.message ?? "解算失败", false);
    return;
  }

  applySolveResultToInputs(panel, result);
  panelState.set(panel.dataset.side, {
    ...sideState,
    solvedResult: result,
    solvedMode: currentMode(panel),
  });

  if (result.command_mode === "joint") {
    setPreview(
      panel,
      `逆解成功: motor1=${format(result.motors.motor_1_position)}, motor2=${format(result.motors.motor_2_position)}`,
      true,
    );
  } else {
    setPreview(
      panel,
      `正解成功: pitch=${format(result.logical.pitch)}, roll=${format(result.logical.roll)}`,
      true,
    );
  }
};

const sendPanel = async (panel) => {
  const sideState = panelState.get(panel.dataset.side) ?? {};
  if (!sideState.solvedResult || sideState.solvedMode !== currentMode(panel)) {
    setMessage(panel.querySelector(".command-message"), "请先解算，再发送", false);
    return;
  }

  const result = await apiPost("/api/send", buildSendPayload(panel, sideState.solvedResult));
  setMessage(panel.querySelector(".command-message"), result.message ?? "发送完成", Boolean(result.success));
  await refreshState();
};

const resetPanel = (panel) => {
  panel.querySelector(".pitch-slider").value = 0;
  panel.querySelector(".roll-slider").value = 0;
  panel.querySelector(".kp-slider").value = 45.0;
  panel.querySelector(".kd-slider").value = 1.4;
  panel.querySelector(".motor1-slider").value = 0;
  panel.querySelector(".motor2-slider").value = 0;
  syncPanelLabels(panel);
  setPreview(panel, "尚未解算", true);
  setMessage(panel.querySelector(".command-message"), "已重置为零位命令", true);
  const state = panelState.get(panel.dataset.side) ?? {};
  panelState.set(panel.dataset.side, { ...state, solvedResult: null, solvedMode: null });
};

const bindPanel = (panel) => {
  const side = panel.dataset.side;
  panelState.set(side, { solvedResult: null, solvedMode: null });

  panel.querySelectorAll("input").forEach((input) => {
    input.addEventListener("input", () => syncPanelLabels(panel));
    input.addEventListener("change", () => syncPanelLabels(panel));
  });

  panel.querySelectorAll(".mode-radio").forEach((radio) => {
    radio.addEventListener("change", () => {
      applyModeState(panel);
      const state = panelState.get(side) ?? {};
      panelState.set(side, { ...state, solvedResult: null, solvedMode: null });
      setPreview(panel, currentMode(panel) === "joint" ? "当前为逆解模式，请先解算" : "当前为正解模式，请先解算", true);
    });
  });

  panel.querySelector(".enable-button").addEventListener("click", async () => {
    const result = await apiPost("/api/enable", { side });
    setMessage(panel.querySelector(".command-message"), result.message ?? "Enabled", Boolean(result.success));
    await refreshState();
  });

  panel.querySelector(".disable-button").addEventListener("click", async () => {
    const result = await apiPost("/api/disable", { side });
    setMessage(panel.querySelector(".command-message"), result.message ?? "Disabled", Boolean(result.success));
    await refreshState();
  });

  panel.querySelector(".solve-button").addEventListener("click", async () => {
    await solvePanel(panel);
  });

  panel.querySelector(".send-button").addEventListener("click", async () => {
    await sendPanel(panel);
  });

  panel.querySelector(".reset-button").addEventListener("click", () => {
    resetPanel(panel);
  });

  syncPanelLabels(panel);
  applyModeState(panel);
  setPreview(panel, "尚未解算", true);
};

panels.forEach(bindPanel);
refreshState();
setInterval(refreshState, 100);
