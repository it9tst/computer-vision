using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace GoAcceleratorEngine
{
    /// <summary>
    /// Command implementation which can execute an Action.
    /// </summary>
    public class CommandAction : ICommand
    {
        private Action _action;
        private bool _canExecute;
        public CommandAction(Action action, bool canExecute)
        {
            _action = action;
            _canExecute = canExecute;
        }

        public virtual bool CanExecute(object parameter)
        {
            return _canExecute;
        }

        public event EventHandler CanExecuteChanged;

        public virtual void Execute(object parameter)
        {
            _action();
        }

        public void RaiseCanExecuteChanged()
        {
            var tmp = CanExecuteChanged;
            if (tmp != null) { tmp(this, EventArgs.Empty); }
        }
    }

    /// <summary>
    /// Command implementation which can execute an Action asynchronously.
    /// </summary>
    public class CommandActionAsync : CommandAction
    {
        public event PropertyChangedEventHandler PropertyChanged;
        protected virtual void RaisePropertyChanged(string propertyName) { RaisePropertyChanged(new PropertyChangedEventArgs(propertyName)); }
        protected virtual void RaisePropertyChanged(PropertyChangedEventArgs args) { var handler = PropertyChanged; if (handler != null) handler(this, args); }

        private bool _isBusy;

        public bool IsBusy
        {
            get { return _isBusy; }
            set
            {
                _isBusy = value;
                RaisePropertyChanged("IsBusy"); 
                RaiseCanExecuteChanged(); 
            }
        }

        public CommandActionAsync(Action action, bool canExecute)
            : base(action, canExecute)
        {
        }

        public override bool CanExecute(object p)
        {
            if (IsBusy) { return false; }

            return base.CanExecute(p);
        }

        public override async void Execute(object p)
        {
            try
            {
                IsBusy = true;
                await Task.Run(() => base.Execute(p));
            }
            finally
            {
                IsBusy = false;
            }
        }
    }
}
