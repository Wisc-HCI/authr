
import {TherbligTypes} from './therblig';

export interface TherbligPrimitive {
  type:TherbligTypes;
  parameters: {[key: string]: any};
  allowed:TherbligTypes[];
  tooltip:string;
  physical:boolean;
  cognitive:boolean;
  sets:{[key:string]: any};
  requires:{[key:string]: any};
}
