import { initializeApp } from "https://www.gstatic.com/firebasejs/12.0.0/firebase-app.js";
import { getDatabase, ref, onValue, set } from "https://www.gstatic.com/firebasejs/12.0.0/firebase-database.js";

const firebaseConfig = {
  apiKey: "AIzaSyDPho5YYOLz2Qg5UIhS3ebiSNxwDuABVlo",
  authDomain: "testhtn-23965.firebaseapp.com",
  databaseURL: "https://testhtn-23965-default-rtdb.firebaseio.com",
  projectId: "testhtn-23965",
  storageBucket: "testhtn-23965.firebasestorage.app",
  messagingSenderId: "1046387453188",
  appId: "1:1046387453188:web:1b2b83f03cac3ffc9b7451"
};

const STATE_PATH = "sensor_logs/latest";
const COMMAND_PATH = "commands";

const app = initializeApp(firebaseConfig);
const db = getDatabase(app);

const userInfo = document.getElementById("userInfo");
const commandStatus = document.getElementById("commandStatus");

const els = {
  mode: document.getElementById("systemMode"),
  fireState: document.getElementById("fireState"),
  temperature: document.getElementById("temperature"),
  humidity: document.getElementById("humidity"),
  mq2: document.getElementById("mq2"),
  flameAo: document.getElementById("flameAo"),
  buzzer: document.getElementById("buzzer"),
  pump: document.getElementById("pump"),
  rgb: document.getElementById("rgb"),
  manualBtn: document.getElementById("manualBtn"),
  resetBtn: document.getElementById("resetBtn"),
  servoAngle: document.getElementById("servoAngle"),
  updatedAt: document.getElementById("updatedAt")
};

function normalizeMode(data) {
  const raw = String(data?.mode ?? "").toUpperCase();

  if (raw === "WARNING" || raw === "WARN") {
    return { text: "WARNING", css: "WARN" };
  }
  if (raw === "ALARM") {
    return { text: "ALARM", css: "ALARM" };
  }
  if (raw === "MANUAL") {
    return { text: "MANUAL", css: "MANUAL" };
  }
  if (raw === "RESET") {
    return { text: "RESET", css: "RESET" };
  }
  if (raw === "SAFE") {
    return { text: "SAFE", css: "SAFE" };
  }

  if (data?.reset) return { text: "RESET", css: "RESET" };
  if (data?.manual || data?.manualBtn) return { text: "MANUAL", css: "MANUAL" };
  if (data?.fullAlarm) return { text: "ALARM", css: "ALARM" };
  if (data?.warning) return { text: "WARNING", css: "WARN" };
  return { text: "SAFE", css: "SAFE" };
}

function formatNumber(value, fallback = 0, digits = 0) {
  const num = Number(value);
  if (!Number.isFinite(num)) return String(fallback);
  return digits > 0 ? num.toFixed(digits) : String(Math.round(num));
}

function setOnOff(el, val) {
  const on = Boolean(val);
  el.textContent = on ? "ON" : "OFF";
  el.className = "metric-value " + (on ? "ok" : "bad");
}

function renderFireState(data) {
  const fire = Boolean(data?.fire);
  els.fireState.textContent = fire ? "FIRE" : "SAFE";
  els.fireState.className = "metric-value " + (fire ? "bad" : "ok");
}

function renderState(data) {
  if (!data) return;

  const mode = normalizeMode(data);
  els.mode.textContent = mode.text;
  els.mode.className = "status-box " + mode.css;

  els.temperature.textContent = `${formatNumber(data.tempC, 0, 1)} C`;
  els.humidity.textContent = `${formatNumber(data.humi, 0, 1)} %`;
  els.mq2.textContent = `${formatNumber(data.mq2AO, 0, 0)}`;
  els.flameAo.textContent = `${formatNumber(data.flameAO, 0, 0)}`;
  els.servoAngle.textContent = `${formatNumber(data.servoAngle, 0, 0)} deg`;
  els.updatedAt.textContent = `updatedAt: ${data.updatedAt ?? data.millis ?? 0}`;

  renderFireState(data);

  setOnOff(els.buzzer, data.buzzer);
  setOnOff(els.pump, data.pump);
  setOnOff(els.rgb, data.rgb);
  setOnOff(els.manualBtn, data.manual ?? data.manualBtn);
  setOnOff(els.resetBtn, data.reset ?? data.resetBtn);
}

async function writeCommands(obj) {
  try {
    const ts = Date.now();
    const cmdId = `cmd_${ts}`;

    const commandPayload = {
      ...obj,
      cmdId,
      sentAt: ts,
      source: "web-dashboard"
    };

    // Chỉ ghi lệnh vào /commands.
    // KHÔNG ghi giả state lên web, để ESP32 cập nhật state thật.
    await set(ref(db, COMMAND_PATH), commandPayload);

    if (commandStatus) {
      commandStatus.textContent = `Đã gửi lệnh lên ${COMMAND_PATH} (cmdId: ${cmdId}). Web sẽ chờ ESP32 cập nhật state thật.`;
      commandStatus.className = "footer-note ok";
    }
  } catch (err) {
    if (commandStatus) {
      commandStatus.textContent = `Lỗi gửi lệnh: ${err.message}`;
      commandStatus.className = "footer-note bad";
    }
    alert("Ghi lệnh thất bại: " + err.message);
  }
}

document.getElementById("pumpOnBtn").onclick = () => writeCommands({ pump: true });
document.getElementById("pumpOffBtn").onclick = () => writeCommands({ pump: false });
document.getElementById("buzzerOnBtn").onclick = () => writeCommands({ buzzer: true });
document.getElementById("buzzerOffBtn").onclick = () => writeCommands({ buzzer: false });
document.getElementById("manualOnBtn").onclick = () => writeCommands({ manualAlarm: true });
document.getElementById("manualOffBtn").onclick = () => writeCommands({ manualAlarm: false });

document.getElementById("resetBtnCmd").onclick = async () => {
  await writeCommands({ resetAlarm: true });
  setTimeout(() => {
    writeCommands({ resetAlarm: false });
  }, 1000);
};

userInfo.textContent = "Public mode";

onValue(ref(db, STATE_PATH), (snapshot) => {
  renderState(snapshot.val());
});