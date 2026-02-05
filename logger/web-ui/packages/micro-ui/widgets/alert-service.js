// src/widgets/alert-service.ts
var AlertService = class {
  alertCallbacks = [];
  register(callback) {
    this.alertCallbacks.push(callback);
  }
  unregister(callback) {
    const index = this.alertCallbacks.indexOf(callback);
    if (index > -1) {
      this.alertCallbacks.splice(index, 1);
    }
  }
  show(options) {
    this.alertCallbacks.forEach((callback) => {
      callback(options);
    });
  }
  showError(message, title = "Error") {
    this.show({
      type: "error",
      icon: "warning",
      title,
      message
    });
  }
  showInfo(message, title = "Information") {
    this.show({
      type: "info",
      icon: "info",
      title,
      message
    });
  }
  showSuccess(message, title = "Success") {
    this.show({
      type: "success",
      icon: "check",
      title,
      message
    });
  }
  showWarning(message, title = "Warning") {
    this.show({
      type: "warning",
      icon: "warning",
      title,
      message
    });
  }
};
var alertService = new AlertService();

export { alertService };
//# sourceMappingURL=alert-service.js.map
//# sourceMappingURL=alert-service.js.map