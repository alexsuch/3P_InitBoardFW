import * as preact from 'preact';

interface ProgressBarProps {
    className?: string;
    height?: number | string;
    value?: number;
    max?: number;
    color?: string;
    label?: string;
    showValue?: boolean;
}
declare function ProgressBar({ className, height, value, max, color, label, showValue }: ProgressBarProps): preact.JSX.Element;

export { ProgressBar, type ProgressBarProps };
