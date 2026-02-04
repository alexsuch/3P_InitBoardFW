import * as preact from 'preact';
import { ComponentChildren } from 'preact';

type SpinnerSize = 'xs' | 's' | 'm' | 'l' | 'xl';
interface SpinnerProps {
    size?: SpinnerSize;
    text?: ComponentChildren;
    className?: string;
    textClassName?: string;
}
declare const Spinner: ({ size, text, className, textClassName }: SpinnerProps) => preact.JSX.Element;

export { Spinner, type SpinnerSize };
