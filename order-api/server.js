import express from "express";
import bodyParser from "body-parser";
import cors from "cors";
import path from "path";
import { fileURLToPath } from "url";

// ---- Firebase Admin ----
import admin from "firebase-admin";
import fs from "fs";

const serviceAccount = JSON.parse(
    fs.readFileSync(new URL("./credential.json", import.meta.url), "utf-8")
);

admin.initializeApp({
  credential: admin.credential.cert(serviceAccount),
});
const db = admin.firestore();

// ---- Express ----
const app = express();
app.use(cors());
app.use(bodyParser.json());

// Serve kiosk UI
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
app.use(express.static(path.join(__dirname, "public")));

// Existing Order Inbox API
let latestOrder = null;
let orderAvailable = false;

function OrderPayload(req, res, role) {
  const { id, items } = req.body;

  if (!id) return res.status(400).json({ error: "Missing id" });
  if (!Array.isArray(items) || items.length === 0) {
    return res.status(400).json({ error: "Empty order" });
  }

  for (const item of items) {
    const required = ["aisle", "name", "price", "rack", "shelf_level", "stock", "qty"];
    for (const key of required) {
      if (!(key in item)) {
        return res.status(400).json({ error: `Missing field '${key}' in item` });
      }
    }
  }

  latestOrder = {
    order_id: Date.now(),
    requester_id: id,
    role,
    items,
    timestamp: new Date().toISOString(),
  };

  orderAvailable = true;

  console.log(`New ${role.toUpperCase()} order received:`);
  console.dir(latestOrder, { depth: null });

  return res.json({ status: "RECEIVED", order_id: latestOrder.order_id });
}

app.post("/api/order/customer", (req, res) => OrderPayload(req, res, "customer"));
app.post("/api/order/employee", (req, res) => OrderPayload(req, res, "employee"));

app.get("/api/order/latest", (req, res) => {
  if (!orderAvailable) return res.status(204).send();
  res.json(latestOrder);
});

app.post("/api/order/ack", (req, res) => {
  console.log("Order acknowledged by ROS");
  orderAvailable = false;
  res.json({ status: "ACKED" });
});

// -------------------------
// Firestore: Accounts
// account/{username}
// fields: access_level, first_name, last_name, password, username
// -------------------------

app.post("/api/auth/login", async (req, res) => {
  try {
    const { username, password } = req.body;
    if (!username || !password) return res.status(400).json({ error: "Missing username/password" });

    const doc = await db.collection("account").doc(username).get();
    if (!doc.exists) return res.status(401).json({ error: "Invalid credentials" });

    const data = doc.data();
    if (data.password !== password) return res.status(401).json({ error: "Invalid credentials" });

    return res.json({
      ok: true,
      username: data.username,
      access_level: data.access_level, // "customer" or "employee"
      first_name: data.first_name || "",
      last_name: data.last_name || "",
    });
  } catch (e) {
    return res.status(500).json({ error: String(e.message || e) });
  }
});

// Create CUSTOMER account (only used by create_customer.html)
app.post("/api/account/create_customer", async (req, res) => {
  try {
    const { username, password, first_name, last_name } = req.body;
    if (!username || !password) return res.status(400).json({ error: "Missing username/password" });

    const ref = db.collection("account").doc(username);
    const existing = await ref.get();
    if (existing.exists) return res.status(409).json({ error: "Username already exists" });

    await ref.set({
      username,
      password,
      first_name: first_name || "",
      last_name: last_name || "",
      access_level: "customer",
    });

    return res.json({ ok: true });
  } catch (e) {
    return res.status(500).json({ error: String(e.message || e) });
  }
});

// Create EMPLOYEE account (only used from employee page)
app.post("/api/account/create_employee", async (req, res) => {
  try {
    const { username, password, first_name, last_name, admin_pin } = req.body;

    // OPTIONAL: protect employee creation with a PIN
    const ADMIN_PIN = process.env.ADMIN_PIN || "";
    if (ADMIN_PIN && admin_pin !== ADMIN_PIN) {
      return res.status(403).json({ error: "Invalid admin PIN" });
    }

    if (!username || !password) return res.status(400).json({ error: "Missing username/password" });

    const ref = db.collection("account").doc(username);
    const existing = await ref.get();
    if (existing.exists) return res.status(409).json({ error: "Username already exists" });

    await ref.set({
      username,
      password,
      first_name: first_name || "",
      last_name: last_name || "",
      access_level: "employee",
    });

    return res.json({ ok: true });
  } catch (e) {
    return res.status(500).json({ error: String(e.message || e) });
  }
});

// -------------------------
// Firestore: Inventory
// grocery_inventory/{docId}
// fields: aisle, name, price, rack, shelf_level, stock
// -------------------------

app.get("/api/inventory/list", async (req, res) => {
  try {
    const snap = await db.collection("grocery_inventory").get();
    const items = [];
    snap.forEach((doc) => items.push({ id: doc.id, ...doc.data() }));
    // sort by name for UI
    items.sort((a, b) => String(a.name || "").localeCompare(String(b.name || "")));
    res.json({ items });
  } catch (e) {
    res.status(500).json({ error: String(e.message || e) });
  }
});

// Employee: update fields (aisle, rack, shelf_level, price) - stock changes handled elsewhere
app.post("/api/inventory/update", async (req, res) => {
  try {
    const { id, aisle, rack, shelf_level, price } = req.body;
    if (!id) return res.status(400).json({ error: "Missing inventory id" });

    const patch = {};
    if (aisle !== undefined) patch.aisle = aisle;
    if (rack !== undefined) patch.rack = rack;
    if (shelf_level !== undefined) patch.shelf_level = shelf_level;
    if (price !== undefined) patch.price = Number(price);

    await db.collection("grocery_inventory").doc(id).update(patch);
    res.json({ ok: true });
  } catch (e) {
    res.status(500).json({ error: String(e.message || e) });
  }
});

// Employee: add new grocery (stock starts at 0)
app.post("/api/inventory/add", async (req, res) => {
  try {
    const { id, name, aisle, rack, shelf_level, price } = req.body;
    if (!id || !name) return res.status(400).json({ error: "Missing id/name" });

    const ref = db.collection("grocery_inventory").doc(id);
    const existing = await ref.get();
    if (existing.exists) return res.status(409).json({ error: "Inventory id already exists" });

    await ref.set({
      name,
      aisle: aisle || "",
      rack: rack ?? 0,
      shelf_level: shelf_level ?? 0,
      price: Number(price ?? 0),
      stock: 0,
    });

    res.json({ ok: true });
  } catch (e) {
    res.status(500).json({ error: String(e.message || e) });
  }
});

// Default route to idle
app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "public", "idle.html"));
});

app.listen(3000, () => {
  console.log("Server running: http://localhost:3000");
});
