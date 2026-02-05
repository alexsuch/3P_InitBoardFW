import * as preact from 'preact';
import { ComponentChildren } from 'preact';

type AlertType = 'info' | 'success' | 'warning' | 'error';
interface AlertProps {
    type?: AlertType;
    title?: string;
    children: ComponentChildren;
    onRetry?: () => void;
    className?: string;
    retryLabel?: string;
}
declare const Alert: ({ type, title, children, onRetry, className, retryLabel }: AlertProps) => preact.JSX.Element;

export { Alert, type AlertProps, type AlertType };
