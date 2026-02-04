// src/widgets/modal-service.ts
var ModalService = class {
  showModalFunction = null;
  hideModalFunction = null;
  register(showModalFn, hideModalFn) {
    this.showModalFunction = showModalFn;
    this.hideModalFunction = hideModalFn;
  }
  show(component, props) {
    if (this.showModalFunction) {
      this.showModalFunction(component, props);
    } else {
      console.error("Modal service is not registered. Did you include ModalProvider?");
    }
  }
  hide() {
    if (this.hideModalFunction) {
      this.hideModalFunction();
    } else {
      console.warn("Modal service hide function is not registered yet. Modal may not close properly.");
    }
  }
};
var modalService = new ModalService();

export { modalService };
//# sourceMappingURL=modal-service.js.map
//# sourceMappingURL=modal-service.js.map