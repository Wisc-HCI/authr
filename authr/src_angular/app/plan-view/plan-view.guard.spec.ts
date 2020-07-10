import { TestBed, async, inject } from '@angular/core/testing';

import { PlanViewGuard } from './plan-view.guard';

describe('PlanViewGuard', () => {
  beforeEach(() => {
    TestBed.configureTestingModule({
      providers: [PlanViewGuard]
    });
  });

  it('should ...', inject([PlanViewGuard], (guard: PlanViewGuard) => {
    expect(guard).toBeTruthy();
  }));
});
