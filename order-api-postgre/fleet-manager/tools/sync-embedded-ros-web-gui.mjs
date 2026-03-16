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
Original upstream: https://github.com/chengyangkj/ros_web_gui_app
Vendored source path: /third_party/ros_web_gui_app
Embedded build target: /order-api-postgre/fleet-manager/public/embedded/ros-web-gui

This folder contains the production build of the vendored ROS Web GUI app used by SlamMapView.
This embedded build includes the English-localized adaptation maintained in this repository.

Open source notice:
本项目采用 CC BY-NC-SA 4.0 许可证（知识共享署名-非商业性使用-相同方式共享 4.0 国际许可协议）。

重要说明：
- 允许：学习、研究、个人使用
- 允许：修改和分发，但必须保留原项目署名
- 禁止：商业用途
- 要求：基于本项目的衍生作品必须采用相同许可证，并附上原项目链接

详情请参阅 /third_party/ros_web_gui_app/LICENSE 文件。

Attribution:
- Original project: https://github.com/chengyangkj/ros_web_gui_app
- Fork used in this repository: https://github.com/StarLionJiang/ros_web_gui_app

Acknowledgements:
- Special thanks to the Lichtblick project for the open-source visualization and message-processing ecosystem, especially @lichtblick/rosmsg and @lichtblick/rosmsg-serialization.
- Thanks to all contributors and users.

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
