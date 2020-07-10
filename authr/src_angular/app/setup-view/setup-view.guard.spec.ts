import { TestBed, async, inject } from '@angular/core/testing';

import { SetupViewGuard } from './setup-view.guard';

describe('SetupViewGuard', () => {
  beforeEach(() => {
    TestBed.configureTestingModule({
      providers: [SetupViewGuard]
    });
  });

  it('should ...', inject([SetupViewGuard], (guard: SetupViewGuard) => {
    expect(guard).toBeTruthy();
  }));
});
