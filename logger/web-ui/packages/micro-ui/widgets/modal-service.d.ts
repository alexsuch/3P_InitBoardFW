type ShowModalFunction = (component: any, props?: any) => void;
declare class ModalService {
    private showModalFunction;
    private hideModalFunction;
    register(showModalFn: ShowModalFunction, hideModalFn: () => void): void;
    show(component: any, props?: any): void;
    hide(): void;
}
declare const modalService: ModalService;

export { modalService };
