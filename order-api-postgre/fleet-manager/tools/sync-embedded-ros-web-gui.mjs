import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const repoRoot = path.resolve(__dirname, "../../..");
const vendorRoot = path.join(repoRoot, "third_party", "ros_web_gui_app");
const vendorDist = path.join(vendorRoot, "dist");
const targetRoot = path.join(repoRoot, "order-api-postgre", "fleet-manager", "public", "embedded", "ros-web-gui");

const vendorNote = `Source repository: https://github.com/StarLionJiang/ros_web_gui_app
Vendored source path: /third_party/ros_web_gui_app
Embedded build target: /order-api-postgre/fleet-manager/public/embedded/ros-web-gui

This folder contains the production build of the vendored ROS Web GUI app used by SlamMapView.
Rebuild it from the vendored source with:
1. npm --prefix third_party/ros_web_gui_app install
2. npm --prefix third_party/ros_web_gui_app run build:embed
3. npm --prefix order-api-postgre/fleet-manager run build:ros-web-gui
`;

if (!fs.existsSync(vendorDist)) {
  throw new Error(`Embedded build not found: ${vendorDist}`);
}

fs.rmSync(targetRoot, { recursive: true, force: true });
fs.mkdirSync(targetRoot, { recursive: true });
fs.cpSync(vendorDist, targetRoot, { recursive: true });
fs.writeFileSync(path.join(targetRoot, "VENDOR.txt"), vendorNote, "utf8");

console.log(`Synced embedded ROS Web GUI to ${targetRoot}`);
