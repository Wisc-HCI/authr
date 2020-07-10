import { Injectable } from '@angular/core';
import { CanActivate, ActivatedRouteSnapshot, RouterStateSnapshot, Router } from '@angular/router';
import { Observable } from 'rxjs';

import { AuthrService } from '../services/authr.service';

@Injectable({
  providedIn: 'root'
})
export class SimulationViewGuard implements CanActivate {
  constructor(public authrService: AuthrService, private router: Router) {}
  canActivate(
    next: ActivatedRouteSnapshot,
    state: RouterStateSnapshot): Observable<boolean> | Promise<boolean> | boolean {
    let url: string = state.url;
    return this.checkLogin(url);
  }

  checkLogin(url: string): boolean {
    if (this.authrService.connected) { return true; }

    this.authrService.redirectUrl = url;

    this.router.navigate(['/login']);
    return false;
  }
}
