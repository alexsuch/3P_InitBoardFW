import { Signal } from '@preact/signals';

declare class MobileViewService {
    private isMobileViewSignal;
    private mediaQuery;
    private changeHandler;
    private initialized;
    init(breakpoint?: string): void;
    dispose(): void;
    get isMobile(): Signal<boolean>;
    /**
     * Manually set the mobile view state.
     * Primarily intended for tests or environments without matchMedia support.
     */
    setIsMobile(isMobile: boolean): void;
}
declare const mobileViewService: MobileViewService;

export { mobileViewService };
