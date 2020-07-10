import { Component, OnInit } from '@angular/core';
import { ActivatedRoute, Router } from '@angular/router';
import { AuthrService } from '../services/authr.service';
import { Task } from '../models/task';
import { MatSnackBar } from '@angular/material';

@Component({
  selector: 'app-task-detail',
  templateUrl: './task-detail.component.html',
  styleUrls: ['./task-detail.component.css']
})

export class TaskDetailComponent implements OnInit {

  public taskKey: string;
  public task: Task;
  private prevTaskName: string;
  private prevRepeat: number;
  private start = true;
  private edited = false;

  constructor(public snackBar: MatSnackBar,
              public route: ActivatedRoute,
              public router: Router,
              public authrService: AuthrService) { }

  ngOnInit() {
      // Set the task based on the route
      this.route.paramMap.subscribe(params => {
          this.taskKey = params.get('task_id');
          this.task = this.authrService.getTaskById(this.taskKey);
      });

  }

  openSnackBar() {
    this.snackBar.open('Macro has been created successfully',null, {
      duration: 500,
    });
  }

  public get taskName() {
    if (this.start === true) {
      this.prevTaskName = this.task.name;
      this.start = !this.start;
    }
    return this.task.name;
  }

  public set taskName(value) {
    this.edited = true;
    this.task.name = value;
  } 

  public get taskRepeat() {
    if (this.start === true) {
      this.prevRepeat = this.task.repeat;
      this.start = !this.start;
    }
    return this.task.repeat;
  }

  public set taskRepeat(value) {
    this.edited = true;
    this.task.repeat = value; 
  }

  private setName() {
    //this.task.name = value;
    this.edited = !this.edited;
    this.authrService.setTaskById({"name":this.task.name},this.taskKey);
    this.authrService.setTaskById({"repeat":this.task.repeat},this.taskKey);
    this.start = !this.start;
  }

  private setRepeat(value) {
    this.task.repeat = value;
    this.authrService.setTaskById({"repeat":this.task.repeat},this.taskKey);
  }

  private cancel() {
    this.task.name = this.prevTaskName;
    this.authrService.setTaskById({"name":this.prevTaskName},this.taskKey);
  }

  private deleteTask(key) {
      this.router.navigate(['/plan']);
      this.authrService.focused = null;
      this.authrService.deleteTask(key);
  }
}
