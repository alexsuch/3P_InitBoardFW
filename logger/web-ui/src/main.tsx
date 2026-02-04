import "./index.css";
import "./app.css";
import { render } from "preact";
import { Button } from "micro-ui/widgets/button";
import { Icon } from "micro-ui/widgets/icon";
import { ModalProvider } from "micro-ui/widgets/modal";
import { ToastProvider } from "micro-ui/widgets/toast-provider";
import { AlertProvider } from "micro-ui/widgets/alert-dialog";
import { injectIconSprite } from "micro-ui/inject-sprite";
import spriteContent from "./assets/icon-sprite.svg?raw";
import { HomePage } from "./pages/HomePage";
import { WiFiPage } from "./pages/WiFiPage";
import { currentPage, menuOpen, navigateTo, connected } from "./store";

// Inject icon sprite
if (
  typeof document !== "undefined" &&
  typeof document.createElement === "function"
) {
  injectIconSprite(spriteContent);
}

const menuItems = [
  { id: "home" as const, label: "Home", icon: "home" },
  { id: "wifi" as const, label: "WiFi", icon: "wifi" },
];

const App = () => {
  const toggleMenu = () => {
    menuOpen.value = !menuOpen.value;
  };

  return (
    <div className="min-h-screen bg-base text-main flex">
      {/* Side Menu */}
      <aside
        className={`
          fixed inset-y-0 left-0 z-50
          w-64 bg-panel border-r border-border
          transform transition-transform duration-300 ease-in-out
          ${menuOpen.value ? "translate-x-0" : "-translate-x-full lg:translate-x-0"}
        `}
      >
        <div className="flex flex-col h-full">
          {/* Menu Header */}
          <div className="flex items-center justify-between p-4 border-b border-border">
            <div className="flex items-center gap-3">
              <Icon name="memory" size="l" />
              <h4 className="text-base font-semibold text-strong">Logger</h4>
            </div>
            <Button
              variant="ghost"
              size="s"
              iconName="close"
              iconOnly
              onClick={toggleMenu}
              className="lg:hidden"
            />
          </div>

          {/* Menu Items */}
          <nav className="flex-1 p-4 space-y-2">
            {menuItems.map((item) => {
              const isActive = currentPage.value === item.id;
              return (
                <div
                  key={item.id}
                  onClick={() => navigateTo(item.id)}
                  className={`
                    flex items-center gap-3 px-4 py-2.5 rounded-lg
                    cursor-pointer transition-all duration-200
                    ${
                      isActive
                        ? "text-primary bg-well"
                        : "text-muted hover:bg-well hover:text-main"
                    }
                  `}
                >
                  <Icon name={item.icon} size="m" />
                  <span className="text-sm font-medium">{item.label}</span>
                </div>
              );
            })}
          </nav>
        </div>
      </aside>

      {/* Overlay for mobile menu */}
      {menuOpen.value && (
        <div
          className="fixed inset-0 bg-black/50 z-40 lg:hidden"
          onClick={toggleMenu}
        />
      )}

      {/* Main Content */}
      <div className="flex-1 flex flex-col min-w-0 lg:ml-64">
        {/* Header */}
        <header className="bg-panel border-b border-border px-4 py-3 sticky top-0 z-30">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              <Button
                variant="ghost"
                size="s"
                iconName="menu"
                iconOnly
                onClick={toggleMenu}
                className="lg:hidden"
              />
              <h4 className="text-lg lg:text-xl font-semibold text-strong">
                {currentPage.value === "home" ? "Home" : "WiFi Settings"}
              </h4>
            </div>
            <div className="flex items-center gap-2">
              {/* Logger Connection Status */}
              <div className="flex items-center gap-2 px-3 py-1.5 rounded-lg bg-well">
                <Icon name="memory" size="s" />
                <span
                  className={`status-dot ${connected.value ? "connected" : "disconnected"}`}
                ></span>
                <span
                  className={`text-xs font-medium ${
                    connected.value ? "text-success" : "text-muted"
                  }`}
                >
                  {connected.value ? "Connected" : "Offline"}
                </span>
              </div>
            </div>
          </div>
        </header>

        {/* Page Content */}
        <main className="flex-1 p-6 overflow-auto">
          {currentPage.value === "home" && <HomePage />}
          {currentPage.value === "wifi" && <WiFiPage />}
        </main>
      </div>
    </div>
  );
};

// Mount the app
if (typeof document !== "undefined") {
  const mountNode = document.getElementById("app");
  if (!mountNode) {
    throw new Error("App mount point (#app) not found");
  }

  render(
    <ModalProvider>
      <AlertProvider>
        <ToastProvider>
          <App />
        </ToastProvider>
      </AlertProvider>
    </ModalProvider>,
    mountNode,
  );
}
