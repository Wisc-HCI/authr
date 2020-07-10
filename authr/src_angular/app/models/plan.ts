export class Plan {
  public name:string;
  public version: string = '0.1';
  public timeweight:number = 1;
  public costweight:number = 1;

  public tasks: string[]    = [];
  public task_lookup        = new Map();
  public therblig_lookup    = new Map();
  public thing_lookup       = new Map();
  public destination_lookup = new Map();
  public agent_lookup       = new Map();
  public tof_lookup         = new Map();

  constructor(name:string) {
    this.name = name;
  }

}
