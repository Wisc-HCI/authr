import { Component, OnInit, OnDestroy } from '@angular/core';
import { AuthrService }                 from "../services/authr.service";
import { MatSnackBar }                  from '@angular/material';

@Component({
  selector: 'setup-agents',
  templateUrl: './setup-agents.component.html',
  styleUrls: ['./setup-agents.component.scss']
})
export class SetupAgentsComponent {

  constructor(public authrService: AuthrService,
              public snackbar: MatSnackBar) { }

}
