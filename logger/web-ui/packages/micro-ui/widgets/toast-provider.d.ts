import * as preact from 'preact';
import { ComponentChildren } from 'preact';

declare const ToastProvider: ({ children }: {
    children: ComponentChildren;
}) => preact.JSX.Element;

export { ToastProvider };
