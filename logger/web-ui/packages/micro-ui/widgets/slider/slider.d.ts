import * as preact from 'preact';
import * as preact_compat from 'preact/compat';
import { Signal } from '@preact/signals';
import { I as IInputProps } from '../../widget-types-CevEJd3U.js';

interface SliderProps extends IInputProps {
    label?: string;
    className?: string;
    id?: string;
    min?: number;
    max?: number;
    step?: number;
    value?: number;
    valueSignal?: Signal<number>;
    disabled?: boolean;
    showValue?: boolean;
}
declare const Slider: preact.FunctionalComponent<preact_compat.PropsWithoutRef<SliderProps> & {
    ref?: preact.Ref<HTMLInputElement> | undefined;
}>;

export { Slider, type SliderProps };
