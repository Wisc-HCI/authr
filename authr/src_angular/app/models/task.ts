export enum TaskType {
  TASK = "task"
}

export interface Task {
  id:string;
  name:string;
  timestamp:number;
  repeat:number;
  therbligs:string[];
}
