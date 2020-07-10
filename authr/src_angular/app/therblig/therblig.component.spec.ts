import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { TherbligComponent } from './therblig.component';

describe('TherbligComponent', () => {
  let component: TherbligComponent;
  let fixture: ComponentFixture<TherbligComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ TherbligComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(TherbligComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
