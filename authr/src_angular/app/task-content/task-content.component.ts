import { AuthrService }     from '../services/authr.service';
import { Route, Router }    from '@angular/router';
import { Component, OnInit} from "@angular/core";

@Component({
  selector: 'app-task-content',
  templateUrl: './task-content.component.html',
  styleUrls: ['./task-content.component.css'],
  host: {'class': 'content-flex'}

})

export class TaskContentComponent implements OnInit {

  constructor(public authrService: AuthrService,
              public router: Router) {
  }

  ngOnInit() {}

  clearDetail(event) {
      this.authrService.focused = null;
      this.router.navigate(['/plan'])
  }
}
