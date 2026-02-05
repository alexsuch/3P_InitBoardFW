// src/widgets/toast-service.ts
var ToastService = class {
  showToastFunction = null;
  hideToastFunction = null;
  register(showFn, hideFn) {
    this.showToastFunction = showFn;
    this.hideToastFunction = hideFn;
  }
  show(options) {
    if (this.showToastFunction) {
      return this.showToastFunction(options);
    }
    console.error("Toast service is not registered. Did you include ToastProvider?");
    return void 0;
  }
  hide(id) {
    if (this.hideToastFunction) {
      this.hideToastFunction(id);
    } else {
      console.warn("Toast service hide function is not registered.");
    }
  }
  showError(message, title = "Error") {
    return this.show({ type: "error", title, message });
  }
  showInfo(message, title = "Info") {
    return this.show({ type: "info", title, message });
  }
  showSuccess(message, title = "Success") {
    return this.show({ type: "success", title, message });
  }
  showWarning(message, title = "Warning") {
    return this.show({ type: "warning", title, message });
  }
};
var toastService = new ToastService();

export { toastService };
//# sourceMappingURL=toast-service.js.map
//# sourceMappingURL=toast-service.js.map