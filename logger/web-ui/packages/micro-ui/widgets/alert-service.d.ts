type AlertType = 'info' | 'success' | 'warning' | 'error';
interface AlertOptions {
    type?: AlertType;
    icon?: string;
    title?: string;
    message: string;
    onOk?: () => void;
}
declare class AlertService {
    private alertCallbacks;
    register(callback: (options: AlertOptions) => void): void;
    unregister(callback: (options: AlertOptions) => void): void;
    show(options: AlertOptions): void;
    showError(message: string, title?: string): void;
    showInfo(message: string, title?: string): void;
    showSuccess(message: string, title?: string): void;
    showWarning(message: string, title?: string): void;
}
declare const alertService: AlertService;

export { alertService };
