const groupsContainer = document.getElementById("groups-container");
const template = document.getElementById("motor-card-template");
const backendStatus = document.getElementById("backend-status");
const motorCount = document.getElementById("motor-count");
const refreshIntervalLabel = document.getElementById("refresh-interval-label");
const refreshButton = document.getElementById("refresh-button");
const stopAllContinuousButton = document.getElementById("stop-all-continuous");
const continuousRateInput = document.getElementById("continuous-rate-hz");

const cardRegistry = new Map();
const continuousTimers = new Map();
let hasRendered = false;
const stateRefreshMs = 100;

const groupTitle = (motor) => {
  if (motor.notes === "torso") {
    return "Torso";
  }
  if (motor.notes === "left_arm") {
    return "Left Arm";
  }
  if (motor.notes === "right_arm") {
    return "Right Arm";
  }
  if (motor.notes === "left_leg") {
    return "Left Leg";
  }
  if (motor.notes === "right_leg") {
    return "Right Leg";
  }
  return "Other";
};

const groupDescription = {
  Torso: "Waist joints and upper trunk drives",
  "Left Arm": "Left shoulder, elbow and wrist chain",
  "Right Arm": "Right shoulder, elbow and wrist chain",
  "Left Leg": "Left hip, knee and ankle chain",
  "Right Leg": "Right hip, knee and ankle chain",
  Other: "Unclassified motors",
};

const stateClass = (status) => {
  if (status === "enabled") {
    return "status-enabled";
  }
  if (status === "disabled") {
    return "status-disabled";
  }
  if (status === "unknown") {
    return "status-unknown";
  }
  return "status-fault";
};

const formatNumber = (value, digits = 3) => Number(value ?? 0).toFixed(digits);

const getSliderRange = (motor, key) => {
  const safety = motor.safety ?? {};
  const limits = motor.limits ?? {};

  if (key === "position") {
    if (safety.has_position_limit) {
      return {
        min: safety.lower_position_limit,
        max: safety.upper_position_limit,
      };
    }
    return {
      min: limits.position_min,
      max: limits.position_max,
    };
  }

  if (key === "velocity") {
    if (Number(safety.max_output_speed) > 0) {
      return {
        min: -Number(safety.max_output_speed),
        max: Number(safety.max_output_speed),
      };
    }
    return {
      min: limits.velocity_min,
      max: limits.velocity_max,
    };
  }

  if (key === "torque") {
    if (Number(safety.max_output_torque) > 0) {
      return {
        min: -Number(safety.max_output_torque),
        max: Number(safety.max_output_torque),
      };
    }
    return {
      min: limits.torque_min,
      max: limits.torque_max,
    };
  }

  return {
    min: limits[`${key}_min`],
    max: limits[`${key}_max`],
  };
};

const getDefaultGainValue = (motor, key) => {
  const defaults = motor.defaults ?? {};
  return Number(defaults[key] ?? 0);
};

const apiPost = async (path, payload) => {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  return response.json();
};

const getContinuousRateMs = () => {
  const hz = Math.max(1, Number(continuousRateInput.value) || 20);
  return Math.round(1000 / hz);
};

const setMessage = (card, text, isSuccess) => {
  const message = card.querySelector(".command-message");
  message.textContent = text;
  message.classList.remove("is-error", "is-success");
  message.classList.add(isSuccess ? "is-success" : "is-error");
};

const renderGroups = (motors) => {
  groupsContainer.innerHTML = "";
  cardRegistry.clear();
  const grouped = new Map();

  motors.forEach((motor) => {
    const key = groupTitle(motor);
    if (!grouped.has(key)) {
      grouped.set(key, []);
    }
    grouped.get(key).push(motor);
  });

  grouped.forEach((groupMotors, groupName) => {
    const section = document.createElement("section");
    section.className = "group-section";
    section.innerHTML = `
      <div class="group-header">
        <div>
          <h2>${groupName}</h2>
          <p>${groupDescription[groupName] ?? ""}</p>
        </div>
        <strong>${groupMotors.length} motors</strong>
      </div>
      <div class="motor-grid"></div>
    `;

    const grid = section.querySelector(".motor-grid");
    groupMotors.forEach((motor) => {
      const card = createMotorCard(motor);
      grid.appendChild(card);
      cardRegistry.set(motor.name, card);
    });
    groupsContainer.appendChild(section);
  });
};

const createSlider = (card, className, limits, key, digits = 3, initialValue = 0, disabled = false) => {
  const slider = card.querySelector(`.${className}`);
  const valueLabel = card.querySelector(`.value-${key}`);
  slider.min = limits.min;
  slider.max = limits.max;
  slider.value = initialValue;
  slider.disabled = disabled;
  valueLabel.textContent = formatNumber(initialValue, digits);
  slider.addEventListener("input", () => {
    valueLabel.textContent = formatNumber(slider.value, digits);
  });
};

const updateCardState = (card, motor) => {
  const state = motor.state ?? {};
  const badge = card.querySelector(".status-badge");
  badge.textContent = state.status ?? "unknown";
  badge.className = `status-badge ${stateClass(state.status ?? "unknown")}`;

  card.querySelector(".state-position").textContent = formatNumber(state.position);
  card.querySelector(".state-velocity").textContent = formatNumber(state.velocity);
  card.querySelector(".state-torque").textContent = formatNumber(state.torque);
  card.querySelector(".state-mos").textContent = formatNumber(state.mos_temperature_c, 1);
  card.querySelector(".state-rotor").textContent = formatNumber(state.rotor_temperature_c, 1);
};

const sendMit = async (name, card) => {
  const payload = {
    name,
    position: Number(card.querySelector(".slider-position").value),
    velocity: Number(card.querySelector(".slider-velocity").value),
    kp: Number(card.querySelector(".slider-kp").value),
    kd: Number(card.querySelector(".slider-kd").value),
    torque: Number(card.querySelector(".slider-torque").value),
  };

  const result = await apiPost("/api/mit", payload);
  setMessage(card, result.message ?? "MIT sent", Boolean(result.success));
  return result;
};

const bindAction = (card, selector, endpoint, successText) => {
  card.querySelector(selector).addEventListener("click", async () => {
    const result = await apiPost(endpoint, { name: card.dataset.name });
    setMessage(card, result.message ?? successText, Boolean(result.success));
    await loadMotors();
  });
};

const stopContinuousSend = (name, card, keepMessage = false) => {
  const timer = continuousTimers.get(name);
  if (timer) {
    clearInterval(timer);
    continuousTimers.delete(name);
  }

  const button = card.querySelector(".toggle-send-button");
  button.textContent = "Continuous Off";
  button.classList.remove("is-active");
  if (!keepMessage) {
    setMessage(card, "Continuous send stopped", true);
  }
};

const startContinuousSend = (name, card) => {
  stopContinuousSend(name, card, true);
  const intervalMs = getContinuousRateMs();
  const timer = setInterval(async () => {
    await sendMit(name, card);
  }, intervalMs);
  continuousTimers.set(name, timer);

  const button = card.querySelector(".toggle-send-button");
  button.textContent = "Continuous On";
  button.classList.add("is-active");
  setMessage(card, `Continuous send running at ${Math.round(1000 / intervalMs)} Hz`, true);
};

const resetMitControls = (card) => {
  ["position", "velocity", "torque"].forEach((key) => {
    const slider = card.querySelector(`.slider-${key}`);
    const valueLabel = card.querySelector(`.value-${key}`);
    slider.value = 0;
    valueLabel.textContent = formatNumber(0);
  });

  ["kp", "kd"].forEach((key) => {
    const slider = card.querySelector(`.slider-${key}`);
    const valueLabel = card.querySelector(`.value-${key}`);
    const defaultValue = Number(card.dataset[`default${key.toUpperCase()}`] ?? 0);
    slider.value = defaultValue;
    valueLabel.textContent = formatNumber(defaultValue);
  });
  setMessage(card, "MIT controls reset to zero", true);
};

const lockGainSlider = (card, key, locked) => {
  const slider = card.querySelector(`.slider-${key}`);
  const button = card.querySelector(`.${key}-lock-button`);
  const defaultValue = Number(card.dataset[`default${key.toUpperCase()}`] ?? 0);

  if (locked) {
    slider.value = defaultValue;
    slider.disabled = true;
    card.querySelector(`.value-${key}`).textContent = formatNumber(defaultValue);
    button.classList.remove("is-unlocked");
  } else {
    slider.disabled = false;
    button.classList.add("is-unlocked");
  }
};

const createMotorCard = (motor) => {
  const fragment = template.content.cloneNode(true);
  const card = fragment.querySelector(".motor-card");
  card.dataset.name = motor.name;
  card.dataset.defaultKP = getDefaultGainValue(motor, "kp");
  card.dataset.defaultKD = getDefaultGainValue(motor, "kd");

  card.querySelector(".motor-group-tag").textContent = motor.interface;
  card.querySelector(".motor-name").textContent = motor.name;
  card.querySelector(".motor-meta").textContent = `${motor.model} | CAN ${motor.can_id}`;
  card.querySelector(".state-interface").textContent = motor.interface;

  createSlider(card, "slider-position", getSliderRange(motor, "position"), "position");
  createSlider(card, "slider-velocity", getSliderRange(motor, "velocity"), "velocity");
  createSlider(card, "slider-kp", getSliderRange(motor, "kp"), "kp", 3, card.dataset.defaultKP, true);
  createSlider(card, "slider-kd", getSliderRange(motor, "kd"), "kd", 3, card.dataset.defaultKD, true);
  createSlider(card, "slider-torque", getSliderRange(motor, "torque"), "torque");

  bindAction(card, ".enable-button", "/api/enable", "Enabled");
  bindAction(card, ".disable-button", "/api/disable", "Disabled");
  bindAction(card, ".clear-button", "/api/clear_error", "Error cleared");
  bindAction(card, ".zero-button", "/api/save_zero", "Zero saved");

  card.querySelector(".send-button").addEventListener("click", async () => {
    await sendMit(motor.name, card);
    await loadMotors();
  });

  card.querySelector(".toggle-send-button").addEventListener("click", async () => {
    if (continuousTimers.has(motor.name)) {
      stopContinuousSend(motor.name, card);
      return;
    }

    const result = await sendMit(motor.name, card);
    if (result.success) {
      startContinuousSend(motor.name, card);
    }
  });

  card.querySelector(".reset-button").addEventListener("click", () => {
    resetMitControls(card);
  });

  card.querySelector(".kp-lock-button").addEventListener("click", () => {
    const slider = card.querySelector(".slider-kp");
    lockGainSlider(card, "kp", !slider.disabled);
  });

  card.querySelector(".kd-lock-button").addEventListener("click", () => {
    const slider = card.querySelector(".slider-kd");
    lockGainSlider(card, "kd", !slider.disabled);
  });

  updateCardState(card, motor);
  return card;
};

const updateExistingCards = (motors) => {
  motors.forEach((motor) => {
    const card = cardRegistry.get(motor.name);
    if (!card) {
      return;
    }
    updateCardState(card, motor);
  });
};

const loadMotors = async () => {
  try {
    const response = await fetch("/api/motors");
    const data = await response.json();
    backendStatus.textContent = "Connected";
    motorCount.textContent = data.motors.length;

    const needsRerender =
      !hasRendered ||
      data.motors.length !== cardRegistry.size ||
      data.motors.some((motor) => !cardRegistry.has(motor.name));

    if (needsRerender) {
      continuousTimers.forEach((timer) => clearInterval(timer));
      continuousTimers.clear();
      renderGroups(data.motors);
      hasRendered = true;
    } else {
      updateExistingCards(data.motors);
    }
  } catch (error) {
    backendStatus.textContent = "Offline";
    console.error(error);
  }
};

const loadStates = async () => {
  try {
    const response = await fetch("/api/states");
    const data = await response.json();
    backendStatus.textContent = "Connected";

    const motors = data.states.map((item) => ({
      name: item.name,
      state: item.state,
    }));
    updateExistingCards(motors);
  } catch (error) {
    backendStatus.textContent = "Offline";
    console.error(error);
  }
};

refreshButton.addEventListener("click", loadMotors);
stopAllContinuousButton.addEventListener("click", () => {
  cardRegistry.forEach((card, name) => {
    if (continuousTimers.has(name)) {
      stopContinuousSend(name, card);
    }
  });
});

continuousRateInput.addEventListener("change", () => {
  const activeNames = Array.from(continuousTimers.keys());
  activeNames.forEach((name) => {
    const card = cardRegistry.get(name);
    if (card) {
      startContinuousSend(name, card);
    }
  });
});

refreshIntervalLabel.textContent = `${stateRefreshMs} ms`;
loadMotors();
setInterval(() => {
  if (hasRendered) {
    loadStates();
  }
}, stateRefreshMs);
