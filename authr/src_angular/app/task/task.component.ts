import { Component, OnInit, OnDestroy, Input, Optional, ChangeDetectionStrategy } from '@angular/core';
import { Task } from '../models/task';
import { Therblig } from '../models/therblig';
import { MatSnackBar } from '@angular/material';
import { AuthrService } from '../services/authr.service';
import { Route, Router } from '@angular/router';

@Component({
  selector: 'task-item',
  templateUrl: './task.component.html',
  styleUrls: ['./task.component.scss']
})
export class TaskComponent implements OnInit {

  @Input() taskKey: string;
  public task: Task;

  constructor(public snackbar: MatSnackBar,
              public router: Router,
              public authrService: AuthrService) { }

  ngOnInit(){
    //this.task= this.authrService.getTaskById(this.taskKey);
    this.task = this.authrService.getTaskById(this.taskKey);
  }

  goToDetail(event) {
      if (this.taskKey !== undefined && this.taskKey  !== null) {
          this.authrService.focused = this.taskKey;
          this.router.navigate(['/plan',{ outlets: { config: ['tasks',this.taskKey] } }]);

      } else {
          this.authrService.focused = null;
          this.router.navigate(['/plan'])
      }
  }

  get repeat(): number {
    return this.authrService.getTaskById(this.taskKey).repeat;
  }

  get name(): string {
    return this.authrService.getTaskById(this.taskKey).name;
  }

  get therbligs(): string[] {
      return this.authrService.getTaskById(this.taskKey).therbligs;
  }

  set therbligs(event){
      this.authrService.setTaskById({"therbligs":event},this.taskKey);
  }
}
