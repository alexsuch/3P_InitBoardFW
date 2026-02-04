import { signal } from '@preact/signals';

// src/widgets/mobile-view-service.ts
var DEFAULT_BREAKPOINT = "(max-width: 768px)";
var MobileViewService = class {
  isMobileViewSignal = signal(false);
  mediaQuery = null;
  changeHandler = null;
  initialized = false;
  init(breakpoint = DEFAULT_BREAKPOINT) {
    if (this.initialized) {
      return;
    }
    if (typeof window === "undefined" || typeof window.matchMedia !== "function") {
      return;
    }
    const mediaQuery = window.matchMedia(breakpoint);
    this.changeHandler = (event) => {
      this.isMobileViewSignal.value = event.matches;
    };
    this.isMobileViewSignal.value = mediaQuery.matches;
    mediaQuery.addEventListener("change", this.changeHandler);
    this.mediaQuery = mediaQuery;
    this.initialized = true;
  }
  dispose() {
    if (!this.mediaQuery) {
      return;
    }
    if (this.changeHandler) {
      this.mediaQuery.removeEventListener("change", this.changeHandler);
    }
    this.mediaQuery = null;
    this.changeHandler = null;
    this.initialized = false;
  }
  get isMobile() {
    return this.isMobileViewSignal;
  }
  /**
   * Manually set the mobile view state.
   * Primarily intended for tests or environments without matchMedia support.
   */
  setIsMobile(isMobile) {
    this.isMobileViewSignal.value = isMobile;
  }
};
var mobileViewService = new MobileViewService();

export { mobileViewService };
//# sourceMappingURL=mobile-view-service.js.map
//# sourceMappingURL=mobile-view-service.js.map