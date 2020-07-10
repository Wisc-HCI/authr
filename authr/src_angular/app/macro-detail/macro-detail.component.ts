import { Component, OnInit } from '@angular/core';
import { ActivatedRoute, Router } from '@angular/router';
import { AuthrService } from '../services/authr.service';
import { Macro } from '../models/macro';

@Component({
  selector: 'app-macro-detail',
  templateUrl: './macro-detail.component.html',
  styleUrls: ['./macro-detail.component.css']
})
export class MacroDetailComponent implements OnInit {

  public macroKey: string;
  public macro: Macro;

  constructor(public route: ActivatedRoute,
              public router: Router,
              public authrService: AuthrService) { }

  ngOnInit() {
      // Set the macro based on the route
      this.route.paramMap.subscribe(params => {
          this.macroKey = params.get('macro_id');
          this.macro = this.authrService.getMacroById(this.macroKey);
      });

  }

  private get name() {
      return this.authrService.getMacroById(this.macroKey).name
  }

  private set name(value) {
      this.authrService.setMacroById({"name":value},this.macroKey)
  }

  private get therbligs() {
      return this.authrService.getMacroById(this.macroKey).therbligs
  }

  private deleteMacro(key) {
      this.router.navigate(['/plan']);
      this.authrService.focused = null;
      this.authrService.deleteMacro(key);
  }

}
