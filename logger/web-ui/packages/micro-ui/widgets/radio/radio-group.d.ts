import * as preact from 'preact';
import { HTMLAttributes } from 'preact/compat';
import { I as IInputProps } from '../../widget-types-CevEJd3U.js';
import { Signal } from '@preact/signals';

interface RadioOption {
    name: string;
    value: any;
}
type OptionType = string | number | RadioOption;
interface RadioGroupProps extends Omit<HTMLAttributes<HTMLDivElement>, 'onChange'>, IInputProps {
    label?: string;
    options: OptionType[];
    value?: any;
    valueSignal?: Signal<any>;
    name: string;
    disabled?: boolean;
    className?: string;
    layout?: 'vertical' | 'horizontal';
    onValidityChange?: (isValid: boolean) => void;
}
declare const RadioGroup: ({ label, options, value, valueSignal, name, disabled, className, onValueChange, onValidityChange, layout, }: RadioGroupProps) => preact.JSX.Element;

export { RadioGroup, type RadioGroupProps, type RadioOption };
